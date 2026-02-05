#include "ecatmaster.h"
#include "cia402.h"
#include "servol7nh.h"

#include <iostream>

/** timeout value in us for return "Operational" state */
#define EC_TIMEOUTOP 50000

#define EC_TIMEOUTCONFIG (EC_TIMEOUTSTATE * 4)

bool EcatMaster::init(const std::string& ifname)
{
    if (!ec_init(ifname.c_str())) {
        std::cout << "[EcatMaster::init] ec_init failed on " << ifname << std::endl;
        return false;
    }

    std::cout << "[EcatMaster::init] ec_init on " << ifname << " succeeded" << std::endl;

    if (ec_config_init(FALSE) <= 0) {
        std::cout << "[EcatMaster::init] No Slaves found" << std::endl;
        ec_close();
        return false;
    }

    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

    std::cout << "[EcatMaster::init] " << ec_slavecount << " slaves found" << std::endl;

    m_Slaves.clear();
    m_Slaves.resize(ec_slavecount + 1);
    for (int i = 1; i <= ec_slavecount; ++i) {
        auto& slave = ec_slave[i];

        // servo L7NH
        if (ServoL7NH::checkL7NH(i)) {
            m_ServoId = i;

            // setup PO2SOconfig function
            slave.PO2SOconfig = &ServoL7NH::setupL7NH;

            // create slave instance
            m_Slaves[i] = std::make_unique<ServoL7NH>(i);
        } else {
            m_Slaves[i] = nullptr;
        }
    }

    ec_config_map(&m_IOmap);
    ec_configdc();

    std::cout << "[EcatMaster::init] Slaves mapped, state to SAFE_OP" << std::endl;

    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTCONFIG);

    m_ExpectedWKC = (ec_group[m_CurrentGroup].outputsWKC * 2) + ec_group[m_CurrentGroup].inputsWKC;
    std::cout << "[EcatMaster::init] Expected WKC : " << m_ExpectedWKC << std::endl;

    return reqOpState();
}

bool EcatMaster::start()
{
    if (m_Running) {
        return false;
    }

    for (int i = 1; i <= ec_slavecount; ++i) {
        if (m_Slaves[i] == nullptr) continue;

        m_Slaves[i]->start();
    }

    m_Running = true;

    m_Worker       = std::thread(&EcatMaster::processLoop, this);
    m_ErrorHandler = std::thread(&EcatMaster::ecatCheck, this);

    return true;
}

void EcatMaster::stop()
{
    if (!m_Running) {
        return;
    }

    m_Running = false;

    if (m_Worker.joinable()) m_Worker.join();
    if (m_ErrorHandler.joinable()) m_ErrorHandler.join();

    for (int i = 1; i <= ec_slavecount; ++i) {
        if (m_Slaves[i] == nullptr) continue;

        m_Slaves[i]->stop();
    }

    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    ec_close();
}

void EcatMaster::servoMovePosition(float ratio)
{
    auto* servo = getPtrServo();

    if (servo == nullptr) {
        return;
    }

    servo->setTargetPosition(ratio);
}

void EcatMaster::setHome()
{
    auto* servo = getPtrServo();

    if (servo == nullptr) {
        return;
    }

    servo->setHome();
}

const ServoStatus& EcatMaster::getServoStatus(int slaveId) const
{
    static constexpr ServoStatus empty {};

    if (ServoL7NH* servo = dynamic_cast<ServoL7NH*>(m_Slaves[slaveId].get())) {
        return servo->getStatus();
    }

    return empty;
}

void EcatMaster::processLoop()
{
    constexpr int cycleTimeUs = 1'000; // 1ms

    while (m_Running) {
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

void EcatMaster::ecatCheck()
{
    constexpr int cycleTimeUs = 10000; // 10ms;

    while (m_Running) {
        if (m_CurrentWKC.load() < m_ExpectedWKC
            || ec_group[m_CurrentGroup].docheckstate) {
            // clear check state flag
            ec_group[m_CurrentGroup].docheckstate = FALSE;
            // read state of all slaves
            ec_readstate();

            slavesCheck();

            // all slaves resumed OP state
            if (!ec_group[m_CurrentGroup].docheckstate) {
                std::cout << "[EcatMaster::ecatCheck] OK : all slaves resumed OPERATIONAL" << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(cycleTimeUs));
    }
}

void EcatMaster::slavesCheck()
{
    for (int i = 1; i <= ec_slavecount; ++i) {
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

ServoL7NH* EcatMaster::getPtrServo()
{
    if (m_ServoId <= 0) {
        return nullptr;
    }

    return dynamic_cast<ServoL7NH*>(m_Slaves[m_ServoId].get());
}
