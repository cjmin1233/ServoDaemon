#include "ecatmaster.h"
#include "servoconfig.h"
#include "servol7nh.h"
#include "servood.h"

#include <QDebug>
#include <iostream>

#ifdef _WIN32
#include <timeapi.h>
#include <windows.h>
#endif

/**
 * @brief Simple DC synchronization helper (from SOEM examples).
 * Adjusts the offset to keep the master cycle synced with the reference slave.
 */
static void ec_sync(int64_t reftime, int64_t cycletime, int64_t* offsettime)
{
    static int64_t integral = 0;

    // 1. 오차(Delta) 계산
    // reftime(슬레이브 시각)을 주기(1ms)로 나눈 나머지 값을 구합니다.
    // 300,000(300us)을 빼는 이유는 네트워크 전송 시간을 고려해
    // 슬레이브의 싱크 펄스보다 마스터가 '약간 일찍' 도착하게 하기 위한 여유값(Margin)입니다.
    int64_t delta = (reftime - 300000) % cycletime;

    // 2. 오차 범위 정규화
    // 나머지가 주기의 절반을 넘어가면, '너무 늦은 것'이 아니라 '너무 빠른 것'으로 해석되도록
    // 오차 범위를 -500us ~ +500us 사이로 맞춥니다.
    if (delta > (cycletime / 2)) delta -= cycletime;

    // 3. 적분항(Integral) 누적
    // 현재 오차가 양수면(늦었으면) 누적값을 키우고, 음수면(빠르면) 누적값을 줄입니다.
    // 이는 아주 미세하게 지속되는 속도 차이(클록 드리프트)를 장기적으로 보정합니다.
    if (delta > 0) integral++;
    if (delta < 0) integral--;

    // 4. 최종 보정값(Offset) 계산 (PI 제어)
    // -(delta / 100) : 비례항(P). 현재 발생한 오차의 1%만큼 즉시 반영합니다.
    // -(integral / 20) : 적분항(I). 누적된 오차를 반영하여 서서히 시계를 맞춥니다.
    // 마이너스가 붙은 이유는 오차가 플러스(지연)일 때 잠자는 시간(sleep)을 줄여야 하기 때문입니다.
    *offsettime = -(delta / 100) - (integral / 20);
}

/** timeout value in us for return "Operational" state */
#define EC_TIMEOUTOP 50000

/** timeout value in us for safe operational state */
#define EC_TIMEOUTCONFIG (EC_TIMEOUTSTATE * 4)

// initialize EtherCAT master, return true if initialized successfully
bool EcatMaster::init(const std::string& ifname)
{
    if (!ec_init(ifname.c_str())) {
        std::cout << "[EcatMaster::init] ec_init failed on " << ifname << std::endl;
        return false;
    }

    // set init flag
    m_Initialized = true;
    // After ec_init succeeded, state should be INIT
    std::cout << "[EcatMaster::init] ec_init on " << ifname << " succeeded" << std::endl;

    if (ec_config_init(FALSE) <= 0) {
        std::cout << "[EcatMaster::init] No Slaves found" << std::endl;
        ec_close();
        m_Initialized = false; // reset init flag

        return false;
    }
    // After ec_config_init succeeded, slaves are in PRE-OP state
    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

    std::cout << "[EcatMaster::init] " << ec_slavecount << " slaves found" << std::endl;

    // create slave instances
    m_Slaves.clear();
    m_Slaves.resize(ec_slavecount + 1);
    for (int i = 1; i <= ec_slavecount; ++i) {
        auto& slave = ec_slave[i];

        // detect and create slave instances
        if (ServoL7NH::checkL7NH(i)) {
            // setup PO2SOconfig function
            slave.PO2SOconfig = &ServoL7NH::setup;

            // create slave instance
            m_Slaves[i] = std::make_unique<ServoL7NH>(i);
        } else {
            m_Slaves[i] = nullptr;
        }
    }

    if (ec_config_map(&m_IOmap) <= 0) {
        std::cout << "[EcatMaster::init] ec_config_map failed" << std::endl;
        ec_close();
        m_Initialized = false;
        return false;
    }

    ec_configdc();

    // After ec_config_map succeeded, slaves are in SAFE-OP state
    std::cout << "[EcatMaster::init] Slaves mapped, state to SAFE_OP" << std::endl;

    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTCONFIG);

    // calculate expected WKC
    m_ExpectedWKC = (ec_group[m_CurrentGroup].outputsWKC * 2) + ec_group[m_CurrentGroup].inputsWKC;
    std::cout << "[EcatMaster::init] Expected WKC : " << m_ExpectedWKC << std::endl;

    if (m_ExpectedWKC <= 0) {
        std::cout << "[EcatMaster::init] Expected WKC is 0. Check slave configurations." << std::endl;
        ec_close();
        m_Initialized = false;
        return false;
    }

    return reqOpState();
}

// start EtherCAT master, return true if started successfully
bool EcatMaster::start()
{
    // already running
    if (m_Running) {
        return false;
    }

    // not initialized
    if (!m_Initialized) {
        return false;
    }

    // Ensure previous threads are properly joined before starting new ones
    if (m_Worker.joinable()) m_Worker.join();
    if (m_ErrorHandler.joinable()) m_ErrorHandler.join();
    if (m_Monitor.joinable()) m_Monitor.join();

    // start all slaves
    for (int i = 1; i <= ec_slavecount; ++i) {
        if (m_Slaves[i] == nullptr) continue;

        m_Slaves[i]->start();
    }

    m_Running = true;

    // start process loop thread, error handler thread
    m_Worker       = std::thread(&EcatMaster::processLoop, this);
    m_ErrorHandler = std::thread(&EcatMaster::ecatCheck, this);
    m_Monitor      = std::thread(&EcatMaster::monitorLoop, this);

    return true;
}

// stop EtherCAT master
void EcatMaster::stop()
{
    m_Running = false;

    // wait for threads to finish
    if (m_Worker.joinable()) m_Worker.join();
    if (m_ErrorHandler.joinable()) m_ErrorHandler.join();
    if (m_Monitor.joinable()) m_Monitor.join();

    // stop all slaves
    for (int i = 1; i <= ec_slavecount; ++i) {
        if (m_Slaves[i] == nullptr) continue;

        m_Slaves[i]->stop();
    }
    m_Slaves.clear();

    if (m_Initialized) {
        // send one last process data to set slaves to INIT state
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);

        // set slaves to INIT state
        ec_slave[0].state = EC_STATE_INIT;
        ec_writestate(0);
        ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTSTATE);

        // close EtherCAT master
        ec_close();

        // reset init flag
        m_Initialized = false;
    }
}

ErrorReason EcatMaster::processCommand(const Command& cmd)
{
    if (!m_Running) {
        return ErrorReason::MasterOffline;
    }

    ServoL7NH* servo = getPtrServo(cmd.slaveId);
    if (servo == nullptr) {
        return ErrorReason::InvalidSlaveId;
    }

    if (servo->getStatus().hasError) {
        return ErrorReason::ServoFault;
    }

    // Command validation passed, enqueue for the process loop
    {
        std::lock_guard<std::mutex> lock(m_cmdMutex);
        m_cmdQueue.push_back(cmd);
    }

    return ErrorReason::None;
}

// if valid servo, return its status; else return empty status
ServoStatus EcatMaster::getServoStatus(int slaveId) const
{
    static constexpr ServoStatus empty {}; // return empty status if invalid

    if (ServoL7NH* servo = dynamic_cast<ServoL7NH*>(m_Slaves[slaveId].get())) {
        return servo->getStatus();
    }

    return empty;
}

bool EcatMaster::isAdapterValid(const std::string& ifname)
{
    if (!ec_init(ifname.c_str())) {
        return false;
    }

    int slaveCount = ec_config_init(FALSE);

    ec_close();
    std::this_thread::sleep_for(std::chrono::microseconds(10000));

    return (slaveCount == ServoConfig::slaveCountMax);
}

// main process loop
void EcatMaster::processLoop()
{
    constexpr int cycleTimeUs = 1'000; // 1ms

#ifdef _WIN32
    timeBeginPeriod(1);
#endif

    while (m_Running) {
        // 1. Process pending commands from TCP (Lock-free swap trick)
        std::vector<Command> localCmds;
        {
            std::lock_guard<std::mutex> lock(m_cmdMutex);
            localCmds.swap(m_cmdQueue);
        }

        for (const auto& cmd : localCmds) {
            int slaveId = cmd.slaveId;
            if (slaveId < 0 || slaveId >= m_Slaves.size()) continue;
            if (m_Slaves[slaveId] == nullptr) continue;

            m_Slaves[slaveId]->processCommand(cmd);
        }

        // 2. process each slave PDO
        for (int i = 1; i <= ec_slavecount; ++i) {
            if (m_Slaves[i] == nullptr) continue;

            m_Slaves[i]->processData();
        }

        ec_send_processdata();
        m_CurrentWKC.store(ec_receive_processdata(EC_TIMEOUTRET));

        if (ec_slavecount > 0) {
            // Calculate DC sync offset (ns)
            ec_sync(ec_DCtime, (int64_t)cycleTimeUs * 1000, &m_syncOffset);
        }

        // sleep with DC adjustment
        int64_t sleepUs = cycleTimeUs + (m_syncOffset / 1000);
        if (sleepUs > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(sleepUs));
        } else {
            // cycle is too late, don't sleep
        }
    }

#ifdef _WIN32
    timeEndPeriod(1);
#endif
}

// request Operational state for all slaves
bool EcatMaster::reqOpState()
{
    ec_slave[0].state = EC_STATE_OPERATIONAL;

    // send one valid process data to make outputs in slaves happy
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    // request OP state for all slaves
    ec_writestate(0);

    // wait for all slaves to reach OP state
    int chk = 200;
    do {
        // Periodic process data to satisfy slave watchdogs during transition
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);

        ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTOP);
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    // check if all slaves are in OP state
    if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
        std::cout << "[EcatMaster::reqOpState] Failed to reach OP state" << std::endl;
        return false;
    }

    std::cout << "[EcatMaster::reqOpState] All slaves in OP state" << std::endl;

    return true;
}

// error handler thread function
void EcatMaster::ecatCheck()
{
    constexpr int cycleTimeUs   = 10000; // 10ms;
    constexpr int errorCountMax = 5;

    int syncCounter = 0;

    while (m_Running) {
        // if WKC is less than expected, or check state flag is set, check all slaves
        int wkc = m_CurrentWKC.load();

        // Force state check every 1 second (100 * 10ms) to ensure state synchronization
        bool forceCheck = (++syncCounter >= 100);

        if (wkc < m_ExpectedWKC || ec_group[m_CurrentGroup].docheckstate || forceCheck) {
            if (forceCheck) syncCounter = 0;

            // clear check state flag
            ec_group[m_CurrentGroup].docheckstate = FALSE;
            // read state of all slaves
            ec_readstate();
            // check each slave state
            slavesCheck();

            // if check state flag is cleared and it wasn't a force check, all slaves are resumed
            if (!ec_group[m_CurrentGroup].docheckstate && !forceCheck) {
                std::cout << "[EcatMaster::ecatCheck] OK : all slaves resumed OPERATIONAL" << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(cycleTimeUs));
    }
}

// check each slave state and try to recover if not in OP state
void EcatMaster::slavesCheck()
{
    for (int i = 1; i <= ec_slavecount; ++i) {
        if (!m_Running) break;

        auto& slave = ec_slave[i];

        if (slave.group == m_CurrentGroup
            && slave.state != EC_STATE_OPERATIONAL) {
            std::cout << "[EcatMaster::slavesCheck] Slave " << i
                      << " state = " << slave.state
                      << " ALStatusCode = " << slave.ALstatuscode << std::endl;

            ec_group[m_CurrentGroup].docheckstate = TRUE;
            // one of the slaves is not in OP state
            if (slave.state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                std::cout << "[EcatMaster::slavesCheck] ERROR : slave " << i
                          << " SAFE_OP + ERROR, request ACK" << std::endl;
                // ACK the error
                slave.state = EC_STATE_PRE_OP + EC_STATE_ACK;
                ec_writestate(i);

                ec_statecheck(i, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

            } else if (slave.state == EC_STATE_INIT) {
                std::cout << "[EcatMaster::slavesCheck] INFO : slave " << i
                          << " INIT -> PRE_OP" << std::endl;
                slave.state = EC_STATE_PRE_OP;
                ec_writestate(i);

                ec_statecheck(i, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
            } else if (slave.state == EC_STATE_PRE_OP) {
                std::cout << "[EcatMaster::slavesCheck] INFO : slave " << i
                          << " PRE_OP -> SAFE_OP" << std::endl;
                slave.state = EC_STATE_SAFE_OP;
                ec_writestate(i);

                ec_statecheck(i, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
            } else if (slave.state == EC_STATE_BOOT) {
                std::cout << "[EcatMaster::slavesCheck] INFO : slave " << i
                          << " BOOT -> INIT" << std::endl;
                slave.state = EC_STATE_INIT;
                ec_writestate(i);

                ec_statecheck(i, EC_STATE_INIT, EC_TIMEOUTSTATE);
            } else if (slave.state == EC_STATE_SAFE_OP) {
                std::cout << "[EcatMaster::slavesCheck] WARNING : slave " << i
                          << " SAFE_OP -> OPERATIONAL" << std::endl;

                // ec_send_processdata();
                // ec_receive_processdata(EC_TIMEOUTRET);

                slave.state = EC_STATE_OPERATIONAL;
                ec_writestate(i);

                ec_statecheck(i, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
            }
            // try to reconfigure the slave
            else if (slave.state > EC_STATE_NONE) {
                if (ec_reconfig_slave(i, EC_TIMEOUTSAFE)) {
                    slave.islost = FALSE;
                    std::cout << "[EcatMaster::slavesCheck] MESSAGE : slave " << i
                              << " reconfigured" << std::endl;
                }
            }
            // check for lost slave
            else if (!slave.islost) {
                ec_statecheck(i, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                if (slave.state == EC_STATE_NONE) {
                    slave.islost = TRUE;
                    std::cout << "[EcatMaster::slavesCheck] ERROR : slave " << i
                              << " lost" << std::endl;
                }
            }
        }

        // slave is lost then try to recover
        if (slave.islost) {
            // check state of lost slave
            if (slave.state != EC_STATE_NONE) {
                slave.islost = FALSE;
                std::cout << "[EcatMaster::slavesCheck] MESSAGE : slave " << i
                          << " found" << std::endl;
                continue;
            }

            // try to recover the slave
            if (ec_recover_slave(i, EC_TIMEOUTSAFE)) {
                slave.islost = FALSE;
                std::cout << "[EcatMaster::slavesCheck] MESSAGE : slave " << i
                          << " recovered" << std::endl;
            }
        }
    }
}

// monitor loop to sdo read from slaves and update their status
void EcatMaster::monitorLoop()
{
    constexpr int cycleTimeUs = 100'000; //  100ms

    while (m_Running) {
        for (int i = 1; i <= ec_slavecount; ++i) {
            int16_t overloadRatio = 0;
            int     size          = sizeof(overloadRatio);

            if (ec_SDOread(i, servoOD::IDX_ACCUMULATED_OVERLOAD, 0, FALSE, &size, &overloadRatio, EC_TIMEOUTRXM) > 0) {
                m_Slaves[i]->setOverloadRatio(overloadRatio);
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(cycleTimeUs));
    }
}

// get pointer to ServoL7NH instance
ServoL7NH* EcatMaster::getPtrServo(int slaveId)
{
    return const_cast<ServoL7NH*>(static_cast<const EcatMaster*>(this)->getPtrServo(slaveId));
}

const ServoL7NH* EcatMaster::getPtrServo(int slaveId) const
{
    if (slaveId <= 0 || slaveId > ec_slavecount) {
        // Out of bounds
        return nullptr;
    }

    return dynamic_cast<const ServoL7NH*>(m_Slaves[slaveId].get());
}
