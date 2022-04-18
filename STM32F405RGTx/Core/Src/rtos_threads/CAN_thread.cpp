#include "main.h"
#include "can.h"
#include "mc.hpp"
#include "threads.hpp"
#include "CAN_thread.hpp"

osMutexId_t g_bus_mutex;
osMessageQueueId_t g_state_change_req_queue;
RTOSThread CANThread::thread_;

void send_frame(CANFrame* frame) {
    if (osMutexAcquire(g_bus_mutex, 0U) != osOK) { Error_Handler(); }
    CANBus_put_frame(frame);
    if (osMutexRelease(g_bus_mutex) != osOK) { Error_Handler(); }
}

void CANThread::initialize() {
    CANThread::thread_ = RTOSThread(
        "CAN_thread",
        1024,
        osPriorityBelowNormal,
        CANThread::runCANThread
    );

    osMutexAttr_t g_bus_mutex_attrs = {
        .name = "bus_mutex",
        .attr_bits = osMutexPrioInherit
    };
    g_bus_mutex = osMutexNew(&g_bus_mutex_attrs);
    if (g_bus_mutex == NULL) { Error_Handler(); }

    g_state_change_req_queue = osMessageQueueNew(10, sizeof(StateID), NULL);
}

void CANThread::send_heartbeat() {
    float avg_fet_temp = (g_mc_data.fet_temps[0] + g_mc_data.fet_temps[1] + g_mc_data.fet_temps[2])/3;

    CANFrame tx_frame0 = CANFrame_init(MOTOR_CONTROLLER_HEALTH_CHECK);
    CANFrame_set_field(&tx_frame0, FET_TEMPERATURE, FLOAT_TO_UINT(avg_fet_temp));
    CANFrame_set_field(&tx_frame0, MOTOR_VOLTAGE, FLOAT_TO_UINT(g_mc_data.dc_voltage));

    CANFrame tx_frame1 = CANFrame_init(MOTOR_CONTROLLER_DATA_1);
    CANFrame_set_field(&tx_frame1, MC_POD_SPEED, FLOAT_TO_UINT(g_mc_data.curr_speed));
    CANFrame_set_field(&tx_frame1, MC_POD_ACCELERATION, g_mc_data.curr_accel);

    CANFrame tx_frame2 = CANFrame_init(MOTOR_CONTROLLER_PHASE_A_STATS);
    CANFrame_set_field(&tx_frame2, PHASE_A_CURRENT, FLOAT_TO_UINT(g_mc_data.pIa));
    CANFrame_set_field(&tx_frame2, PHASE_A_VOLTAGE, FLOAT_TO_UINT(g_mc_data.pVa));

    CANFrame tx_frame3 = CANFrame_init(MOTOR_CONTROLLER_PHASE_B_STATS);
    CANFrame_set_field(&tx_frame3, PHASE_A_CURRENT, FLOAT_TO_UINT(g_mc_data.pIb));
    CANFrame_set_field(&tx_frame3, PHASE_A_VOLTAGE, FLOAT_TO_UINT(g_mc_data.pVb));

    CANFrame tx_frame4 = CANFrame_init(MOTOR_CONTROLLER_PHASE_C_STATS);
    CANFrame_set_field(&tx_frame4, PHASE_C_CURRENT, FLOAT_TO_UINT(g_mc_data.pIc));
    CANFrame_set_field(&tx_frame4, PHASE_C_VOLTAGE, FLOAT_TO_UINT(g_mc_data.pVc));

    send_frame(&tx_frame0);
    send_frame(&tx_frame1);
    send_frame(&tx_frame2);
    send_frame(&tx_frame3);
    send_frame(&tx_frame4);
}

void CANThread::runCANThread(void* arg) {
    // send a BUS_TEST_REQ, if we don't get a response we are disconnected from the bus...
    CANFrame req_frame = CANFrame_init(MOTOR_CONTROLLER_REQ);
    send_frame(&req_frame);

    // wait for a response...
    uint32_t start_time = HAL_GetTick();
    while (!Queue_empty(&RX_QUEUE)) {
        if ((HAL_GetTick() - start_time) > BUS_TEST_REQ_TIMEOUT) { Error_Handler(); }
    }

    while (1) {
        if (!Queue_empty(&RX_QUEUE)) {
            CANFrame rx_frame = CANBus_get_frame();
            switch (rx_frame.id) {
                case STATE_CHANGE_REQ : {
                    StateID state_change_req_id = (StateID)CANFrame_get_field(&rx_frame, STATE_ID);
                    if (state_change_req_id == AUTO_PILOT) {
                        g_mc_data.track_length = CANFrame_get_field(&rx_frame, TRACK_LENGTH);
                    }
                    osMessageQueuePut(g_state_change_req_queue, &state_change_req_id, 0, 0);
                    break;
                }
                case RING_ENCODER_DATA : {
                    g_mc_data.ring_encoder_reported_speed = UINT_TO_FLOAT(CANFrame_get_field(&rx_frame, RE_POD_SPEED));
                    break;
                }
                case MANUAL_CONTROL_1 : {
                    if (StateMachineThread::getState() != ManualControl) { Error_Handler(); }

                    // TODO: implement the below pseudocode
                    // PIDThread::set_val = UINT_TO_FLOAT(CANFrame_get_field(&rx_frame, TARGET_SPEED))
                    // SVPWMThread::fundamental_freq = UINT_TO_FLOAT(CANFrame_get_field(&rx_frame, TARGET_FREQUENCY))
                    break;
                }
                case MANUAL_CONTROL_2 : {
                    if (StateMachineThread::getState() != ManualControl) { Error_Handler(); }

                    float target_power = UINT_TO_FLOAT(CANFrame_get_field(&rx_frame, TARGET_POWER));
                    // TODO: set the mod index in the SVPWMThread to achieve the target power...
                    break;
                }
                case MANUAL_CONTROL_3 : {
                    if (StateMachineThread::getState() != ManualControl) { Error_Handler(); }

                    StateMachineThread::current_limit = UINT_TO_FLOAT(CANFrame_get_field(&rx_frame, SET_TEMPERATURE_LIMIT));
                    StateMachineThread::temperature_limit = UINT_TO_FLOAT(CANFrame_get_field(&rx_frame, SET_CURRENT_LIMIT));
                    break;
                }
            }
        }

        CANThread::send_heartbeat();
        osDelay(CAN_THREAD_PERIODICITY);
    }
}

