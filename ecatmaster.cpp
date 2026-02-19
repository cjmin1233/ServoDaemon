#include "ecatmaster.h"
#include "cia402.h"
#include "servol7nh.h"

#include <iostream>

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
            m_ServoId = i;

            // setup PO2SOconfig function
            slave.PO2SOconfig = &ServoL7NH::setup;

            // create slave instance
            m_Slaves[i] = std::make_unique<ServoL7NH>(i);
        } else {
            m_Slaves[i] = nullptr;
        }
    }

    ec_config_map(&m_IOmap);
    ec_configdc();

    // After ec_config_map succeeded, slaves are in SAFE-OP state
    std::cout << "[EcatMaster::init] Slaves mapped, state to SAFE_OP" << std::endl;

    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTCONFIG);

    // calculate expected WKC
    m_ExpectedWKC = (ec_group[m_CurrentGroup].outputsWKC * 2) + ec_group[m_CurrentGroup].inputsWKC;
    std::cout << "[EcatMaster::init] Expected WKC : " << m_ExpectedWKC << std::endl;

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

    // start all slaves
    for (int i = 1; i <= ec_slavecount; ++i) {
        if (m_Slaves[i] == nullptr) continue;

        // TEST
        if (i > 1) continue;

        m_Slaves[i]->start();
    }

    m_Running = true;

    // start process loop thread, error handler thread
    m_Worker       = std::thread(&EcatMaster::processLoop, this);
    m_ErrorHandler = std::thread(&EcatMaster::ecatCheck, this);

    return true;
}

// stop EtherCAT master
void EcatMaster::stop()
{
    m_Running = false;

    // wait for threads to finish
    if (m_Worker.joinable()) m_Worker.join();
    if (m_ErrorHandler.joinable()) m_ErrorHandler.join();

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

// set target position to servo in ratio [0.0, 1.0]
void EcatMaster::servoMovePosition(float ratio)
{
    // clamp ratio to [0.0, 1.0]
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;

    // get pointer to servo
    auto* servo = getPtrServo();

    if (servo == nullptr) {
        return;
    }

    servo->setTargetPosition(ratio);
}

// set homing command to servo
void EcatMaster::setHome()
{
    // get pointer to servo
    auto* servo = getPtrServo();

    if (servo == nullptr) {
        return;
    }

    servo->setHome();
}

void EcatMaster::setTorque()
{
    // get pointer to servo
    auto* servo = getPtrServo();

    if (servo == nullptr) {
        return;
    }

    servo->setTorque(1000); // 100% torque for test
}

// if valid servo, return its status; else return empty status
const ServoStatus& EcatMaster::getServoStatus(int slaveId) const
{
    static constexpr ServoStatus empty {}; // return empty status if invalid

    if (ServoL7NH* servo = dynamic_cast<ServoL7NH*>(m_Slaves[slaveId].get())) {
        return servo->getStatus();
    }

    return empty;
}

const bool EcatMaster::isServoRunning() const
{
    if (const auto* ptrServo = getPtrServo()) {
        return ptrServo->isRunning();
    }
    return false;
}

bool EcatMaster::isAdapterValid(const std::string& ifname)
{
    if (!ec_init(ifname.c_str())) {
        return false;
    }

    int slaveCount = ec_config_init(FALSE);

    ec_close();
    std::this_thread::sleep_for(std::chrono::microseconds(10000));

    return (slaveCount > 0);
}

// main process loop
void EcatMaster::processLoop()
{
    constexpr int cycleTimeUs = 1'000; // 1ms

    while (m_Running) {
        // process each slave
        for (int i = 1; i <= ec_slavecount; ++i) {
            if (m_Slaves[i] == nullptr) continue;

            m_Slaves[i]->processData();
        }

        ec_send_processdata();
        m_CurrentWKC.store(ec_receive_processdata(EC_TIMEOUTRET));
        // sleep
        std::this_thread::sleep_for(std::chrono::microseconds(cycleTimeUs));
    }
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

    int continuousErrorCount = 0;

    while (m_Running) {
        // if WKC is less than expected, or check state flag is set, check all slaves
        if (m_CurrentWKC.load() < m_ExpectedWKC
            || ec_group[m_CurrentGroup].docheckstate) {
            // increment continuous error count
            ++continuousErrorCount;

            // if error count exceeds max, stop the master
            if (continuousErrorCount > errorCountMax) {
                std::cerr << "[EcatMaster::ecatCheck] Critical Link Loss Detected!" << std::endl;

                m_Running = false;
                break;
            }

            // clear check state flag
            ec_group[m_CurrentGroup].docheckstate = FALSE;
            // read state of all slaves
            ec_readstate();
            // check each slave state
            slavesCheck();

            // if check state flag is cleared, all slaves are resumed to OP state
            if (!ec_group[m_CurrentGroup].docheckstate) {
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

// get pointer to ServoL7NH instance
ServoL7NH* EcatMaster::getPtrServo()
{
    return const_cast<ServoL7NH*>(static_cast<const EcatMaster*>(this)->getPtrServo());
}

const ServoL7NH* EcatMaster::getPtrServo() const
{
    if (m_ServoId <= 0) {
        return nullptr;
    }

    return dynamic_cast<const ServoL7NH*>(m_Slaves[m_ServoId].get());
}
