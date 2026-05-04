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

/** timeout value in us for return "Operational" state */
#define EC_TIMEOUTOP 50000

/** timeout value in us for safe operational state */
#define EC_TIMEOUTCONFIG (EC_TIMEOUTSTATE * 4)

/**
 * @brief Simple DC synchronization helper (from SOEM examples).
 * Adjusts the offset to keep the master cycle synced with the reference slave.
 */
static void ec_sync(int64_t reftime, int64_t cycletime, int64_t* offsettime)
{
    static int64_t integral = 0;

    // 1. Calculate Delta
    // Get the remainder of reftime (slave time) divided by the period (1ms).
    // Subtracting 300,000 (300us) to ensure the master arrives slightly earlier than the slave's sync pulse,
    // considering network latency (margin).
    int64_t delta = (reftime - 300000) % cycletime;

    // 2. Normalize Delta Range
    // If the remainder exceeds half the period, interpret it as "too fast" instead of "too slow",
    // and normalize the range to [-500us, +500us].
    if (delta > (cycletime / 2)) delta -= cycletime;

    // 3. Accumulate Integral Term
    // If current delta is positive (late), increase integral; if negative (fast), decrease integral.
    // This corrects long-term clock drift (steady-state error).
    if (delta > 0) integral++;
    if (delta < 0) integral--;

    // 4. Calculate Final Offset (PI Control)
    // -(delta / 100): Proportional (P) term. Adjusts 1% of the current error immediately.
    // -(integral / 20): Integral (I) term. Adjusts based on accumulated error for stable synchronization.
    // Negative sign is used because if delta is positive (late), we need to decrease sleep duration.
    *offsettime = -(delta / 100) - (integral / 20);
}

// Initialize EtherCAT master, return true if initialized successfully
bool EcatMaster::init(const std::string& ifname)
{
    if (!ec_init(ifname.c_str())) {
        std::cout << "[EcatMaster::init] ec_init failed on " << ifname << std::endl;
        return false;
    }

    // Set init flag
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

    // Create slave instances
    m_Slaves.clear();
    m_Slaves.resize(ec_slavecount + 1);
    for (int i = 1; i <= ec_slavecount; ++i) {
        auto& slave = ec_slave[i];

        // Detect and create slave instances
        if (ServoL7NH::checkL7NH(i)) {
            // Setup PO2SOconfig function
            slave.PO2SOconfig = &ServoL7NH::setup;

            // Create slave instance
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

    // Calculate expected WKC
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

// Start EtherCAT master, return true if started successfully
bool EcatMaster::start()
{
    // Already running
    if (m_Running) {
        return false;
    }

    // Not initialized
    if (!m_Initialized) {
        return false;
    }

    // Ensure previous threads are properly joined before starting new ones
    if (m_Worker.joinable()) m_Worker.join();
    if (m_ErrorHandler.joinable()) m_ErrorHandler.join();
    if (m_Monitor.joinable()) m_Monitor.join();

    // Start all slaves
    for (int i = 1; i <= ec_slavecount; ++i) {
        if (m_Slaves[i] == nullptr) continue;

        m_Slaves[i]->start();
    }

    m_Running = true;

    // Start process loop thread, error handler thread, and monitor thread
    m_Worker       = std::thread(&EcatMaster::processLoop, this);
    m_ErrorHandler = std::thread(&EcatMaster::ecatCheck, this);
    m_Monitor      = std::thread(&EcatMaster::monitorLoop, this);

    return true;
}

// Stop EtherCAT master
void EcatMaster::stop()
{
    m_Running = false;

    // Wait for threads to finish
    if (m_Worker.joinable()) m_Worker.join();
    if (m_ErrorHandler.joinable()) m_ErrorHandler.join();
    if (m_Monitor.joinable()) m_Monitor.join();

    // Stop all slaves
    for (int i = 1; i <= ec_slavecount; ++i) {
        if (m_Slaves[i] == nullptr) continue;

        m_Slaves[i]->stop();
    }
    m_Slaves.clear();

    if (m_Initialized) {
        // Send one last process data to set slaves to INIT state
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);

        // Set slaves to INIT state
        ec_slave[0].state = EC_STATE_INIT;
        ec_writestate(0);
        ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTSTATE);

        // Close EtherCAT master
        ec_close();

        // Reset init flag
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

// If valid servo, return its status; else return empty status
ServoStatus EcatMaster::getServoStatus(int slaveId) const
{
    static constexpr ServoStatus empty {}; // Return empty status if invalid

    if (slaveId <= 0 || slaveId > ec_slavecount) {
        return empty;
    }

    if (ServoL7NH* servo = dynamic_cast<ServoL7NH*>(m_Slaves[slaveId].get())) {
        return servo->getStatus();
    }

    return empty;
}

// Check if the adapter is valid by attempting to initialize and count slaves
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

// Request Operational state for all slaves
bool EcatMaster::reqOpState()
{
    ec_slave[0].state = EC_STATE_OPERATIONAL;

    // Send one valid process data to make outputs in slaves happy
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    // Request OP state for all slaves
    ec_writestate(0);

    // Wait for all slaves to reach OP state
    int chk = 200;
    do {
        // Periodic process data to satisfy slave watchdogs during transition
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);

        ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTOP);
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    // Check if all slaves are in OP state
    if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
        std::cout << "[EcatMaster::reqOpState] Failed to reach OP state" << std::endl;
        return false;
    }

    std::cout << "[EcatMaster::reqOpState] All slaves in OP state" << std::endl;

    return true;
}

// Main process loop for PDO exchange and DC sync
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

        // 2. Process each slave PDO
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

        // Sleep with DC adjustment
        int64_t sleepUs = cycleTimeUs + (m_syncOffset / 1000);
        if (sleepUs > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(sleepUs));
        } else {
            // Cycle is too late, don't sleep
        }
    }

#ifdef _WIN32
    timeEndPeriod(1);
#endif
}

// Error handler thread function to monitor WKC and slave states
void EcatMaster::ecatCheck()
{
    constexpr int cycleTimeUs   = 10000; // 10ms;
    constexpr int errorCountMax = 5;

    int syncCounter = 0;

    while (m_Running) {
        // If WKC is less than expected, or check state flag is set, check all slaves
        int wkc = m_CurrentWKC.load();

        // Force state check every 1 second (100 * 10ms) to ensure state synchronization
        bool forceCheck = (++syncCounter >= 100);

        if (wkc < m_ExpectedWKC || ec_group[m_CurrentGroup].docheckstate || forceCheck) {
            if (forceCheck) syncCounter = 0;

            // Clear check state flag
            ec_group[m_CurrentGroup].docheckstate = FALSE;
            // Read state of all slaves
            ec_readstate();
            // Check each slave state
            slavesCheck();

            // If check state flag is cleared and it wasn't a force check, all slaves are resumed
            if (!ec_group[m_CurrentGroup].docheckstate && !forceCheck) {
                std::cout << "[EcatMaster::ecatCheck] OK : all slaves resumed OPERATIONAL" << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(cycleTimeUs));
    }
}

// Check each slave state and try to recover if not in OP state
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
            // One of the slaves is not in OP state
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
            // Try to reconfigure the slave
            else if (slave.state > EC_STATE_NONE) {
                if (ec_reconfig_slave(i, EC_TIMEOUTSAFE)) {
                    slave.islost = FALSE;
                    std::cout << "[EcatMaster::slavesCheck] MESSAGE : slave " << i
                              << " reconfigured" << std::endl;
                }
            }
            // Check for lost slave
            else if (!slave.islost) {
                ec_statecheck(i, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                if (slave.state == EC_STATE_NONE) {
                    slave.islost = TRUE;
                    std::cout << "[EcatMaster::slavesCheck] ERROR : slave " << i
                              << " lost" << std::endl;
                }
            }
        }

        // Slave is lost then try to recover
        if (slave.islost) {
            // Check state of lost slave
            if (slave.state != EC_STATE_NONE) {
                slave.islost = FALSE;
                std::cout << "[EcatMaster::slavesCheck] MESSAGE : slave " << i
                          << " found" << std::endl;
                continue;
            }

            // Try to recover the slave
            if (ec_recover_slave(i, EC_TIMEOUTSAFE)) {
                slave.islost = FALSE;
                std::cout << "[EcatMaster::slavesCheck] MESSAGE : slave " << i
                          << " recovered" << std::endl;
            }
        }
    }
}

// Monitor loop to read SDO from slaves and update their status (e.g., overload ratio)
void EcatMaster::monitorLoop()
{
    constexpr int cycleTimeUs = 100'000; // 100ms

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

// Get pointer to ServoL7NH instance (non-const)
ServoL7NH* EcatMaster::getPtrServo(int slaveId)
{
    return const_cast<ServoL7NH*>(static_cast<const EcatMaster*>(this)->getPtrServo(slaveId));
}

// Get pointer to ServoL7NH instance (const)
const ServoL7NH* EcatMaster::getPtrServo(int slaveId) const
{
    if (slaveId <= 0 || slaveId > ec_slavecount) {
        // Out of bounds
        return nullptr;
    }

    return dynamic_cast<const ServoL7NH*>(m_Slaves[slaveId].get());
}
