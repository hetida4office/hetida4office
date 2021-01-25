#include "mqttsn/fg_mqttsn_actor.h"
#include "bsp/fg_bsp_actor.h"
#include "gpio/fg_gpio.h"
#include "pins.h"
#include <app_scheduler.h>
#include <app_timer.h>
#include <nrf_assert.h>
#include <nrf_atfifo.h>
#include <nrfx.h>

#include <bsp_thread.h>
#include <openthread/thread.h>
#include <thread_utils.h>

#include <mqttsn_client.h>


/** Logging */
#define NRFX_FG_MQTTSN_ACTOR_CONFIG_LOG_ENABLED 1
#define NRFX_FG_MQTTSN_ACTOR_CONFIG_LOG_LEVEL 4
#define NRFX_FG_MQTTSN_ACTOR_CONFIG_INFO_COLOR 7
#define NRFX_FG_MQTTSN_ACTOR_CONFIG_DEBUG_COLOR 7
#define NRFX_LOG_MODULE FG_MQTTSN_ACTOR
#include <nrfx_log.h>


/** Actor resources */
FG_ACTOR_SLOTS_DEC(fg_mqttsn_connect, fg_mqttsn_publish, fg_mqttsn_sleep, fg_mqttsn_disconnect);

#define FG_MQTTSN_ACTOR_SLOT_ASSIGNMENT                                                            \
    {                                                                                              \
        [FG_MQTTSN_CONNECT] = fg_mqttsn_connect, [FG_MQTTSN_DISCONNECT] = fg_mqttsn_disconnect,    \
        [FG_MQTTSN_PUBLISH] = fg_mqttsn_publish, [FG_MQTTSN_SLEEP] = fg_mqttsn_sleep               \
    }
#define FG_MQTTSN_ACTOR_STATES                                                                     \
    {                                                                                              \
        MQTTSN_UNINITIALIZED, MQTTSN_KEEP_CONNECTED, MQTTSN_KEEP_ASLEEP, MQTTSN_KEEP_DISCONNECTED  \
    }

FG_ACTOR_DEF(FG_MQTTSN_ACTOR_SLOT_ASSIGNMENT, FG_MQTTSN_ACTOR_STATES, FG_ACTOR_SINGLETON_TASK);

FG_ACTOR_TASK_CALLBACKS_DEC(fg_mqttsn_connect_cb, fg_mqttsn_sleep_cb, fg_mqttsn_disconnect_cb);

FG_ACTOR_INTERFACE_LOCAL_DEC();

#define SCHED_QUEUE_SIZE 32 // Maximum number of events in the scheduler queue.
#define SCHED_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE // Maximum app_scheduler event size.

static otInstance * m_p_fg_mqttsn_thread_instance;

#define FG_MQTT_THREAD_CHILD_TIMEOUT                                                               \
    65 // Thread child timeout [s] - must be higher than the max polling period.
#define FG_MQTT_THREAD_SLEEP_TIMEOUT                                                               \
    120 // Interval between MQTTSN gateway ping packages while sleeping [s].
#define FG_MQTT_THREAD_AWAKE_POLL_PERIOD                                                           \
    200 // Thread Sleepy End Device polling period when MQTT-SN Awake. [ms]
#define FG_MQTT_SEARCH_GATEWAY_TIMEOUT 5 // MQTT-SN Gateway discovery procedure timeout in [s].
#define FG_MQTTSN_MAX_FAILED_PUBACK_BEFORE_RECONNECT 3
static uint32_t m_fg_requested_sleep_poll_period; // Thread Sleepy End Device polling period when
                                                  // MQTT-SN Asleep. [ms]

static char m_fg_mqttsn_client_id[] =
    "fg_h4o_" STRINGIFY(FG_MQTT_SENSOR_NAME); // TODO: pass in as parameter to init call.
static mqttsn_client_t m_fg_mqttsn_client;

static mqttsn_connect_opt_t m_fg_mqttsn_connect_opt;

static mqttsn_remote_t m_fg_mqttsn_gateway_addr;
static uint8_t m_fg_mqttsn_gateway_id;

#define FG_MQTT_TOPIC_NAME(topic) "h4o/" STRINGIFY(FG_MQTT_SENSOR_NAME) "/" STRINGIFY(topic)

static const char * const m_fg_mqttsn_topic_names[FG_MQTT_TOPIC_NUM] = {FG_MQTT_TOPIC_NAME(press),
    FG_MQTT_TOPIC_NAME(temp), FG_MQTT_TOPIC_NAME(hum), FG_MQTT_TOPIC_NAME(co2),
    FG_MQTT_TOPIC_NAME(bat), FG_MQTT_TOPIC_NAME(msg)}; // TODO: pass in as parameter to init call.

static mqttsn_topic_t m_fg_mqttsn_topics[FG_MQTT_TOPIC_NUM] = {
    {
        .p_topic_name = m_fg_mqttsn_topic_names[FG_MQTT_TOPIC_PRESSURE],
    },
    {
        .p_topic_name = m_fg_mqttsn_topic_names[FG_MQTT_TOPIC_TEMPERATURE],
    },
    {
        .p_topic_name = m_fg_mqttsn_topic_names[FG_MQTT_TOPIC_HUMIDITY],
    },
    {
        .p_topic_name = m_fg_mqttsn_topic_names[FG_MQTT_TOPIC_CO2],
    },
    {
        .p_topic_name = m_fg_mqttsn_topic_names[FG_MQTT_TOPIC_BAT],
    },
    {
        .p_topic_name = m_fg_mqttsn_topic_names[FG_MQTT_TOPIC_STATUS],
    }};

#define FG_MQTTSN_MAX_CONCURRENT_MESSAGES 10

static uint16_t m_fg_mqttsn_msg_id = 0;
NRF_ATFIFO_DEF(
    m_p_fg_mqttsn_message_fifo, fg_mqttsn_message_t *, FG_MQTTSN_MAX_CONCURRENT_MESSAGES);
static uint8_t m_fg_num_concurrent_messages = 0;


/** Public API */
FG_ACTOR_INIT_DEF(mqttsn, MQTTSN_UNINITIALIZED, MQTTSN_KEEP_DISCONNECTED, fg_mqttsn_init);


/** Internal implementation */
static void fg_mqttsn_init(void)
{
    NRF_ATFIFO_INIT(m_p_fg_mqttsn_message_fifo);

    APP_SCHED_INIT(SCHED_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    fg_bsp_actor_init();
    bsp_board_led_on(BSP_BOARD_LED_1);
}

static void fg_mqttsn_thread_state_changed_cb(uint32_t flags, void * p_context);

static ret_code_t fg_mqttsn_thread_init(void)
{
    thread_configuration_t thread_configuration = {
        .radio_mode = THREAD_RADIO_MODE_RX_OFF_WHEN_IDLE,
        .autocommissioning = true,
        .poll_period = FG_MQTT_THREAD_AWAKE_POLL_PERIOD,
        .default_child_timeout = FG_MQTT_THREAD_CHILD_TIMEOUT,
    };

    thread_init(&thread_configuration);
    thread_cli_init();
    thread_state_changed_callback_set(fg_mqttsn_thread_state_changed_cb);
    return bsp_thread_init(thread_ot_instance_get());
}

static void fg_mqttsn_evt_handler(mqttsn_client_t * p_client, mqttsn_event_t * p_event);

static void fg_mqttsn_connect_opt_init(void)
{
    m_fg_mqttsn_connect_opt.alive_duration = MQTTSN_DEFAULT_ALIVE_DURATION;
    m_fg_mqttsn_connect_opt.clean_session = MQTTSN_DEFAULT_CLEAN_SESSION_FLAG;
    m_fg_mqttsn_connect_opt.will_flag = MQTTSN_DEFAULT_WILL_FLAG;
    m_fg_mqttsn_connect_opt.client_id_len = strlen(m_fg_mqttsn_client_id);
    memcpy(m_fg_mqttsn_connect_opt.p_client_id, (unsigned char *)m_fg_mqttsn_client_id,
        m_fg_mqttsn_connect_opt.client_id_len);
}

FG_ACTOR_SLOT(fg_mqttsn_connect)
{
    ASSERT(!m_p_fg_mqttsn_thread_instance)
    FG_ACTOR_STATE_TRANSITION(
        MQTTSN_KEEP_DISCONNECTED, MQTTSN_KEEP_CONNECTED, "connecting MQTTSN client");

    FG_ACTOR_RUN_SINGLETON_TASK(fg_mqttsn_connect_cb);

    DRVX(fg_mqttsn_thread_init()); // Obs: starts thread auto-connection which guarantees progress.
    m_p_fg_mqttsn_thread_instance = thread_ot_instance_get();

    DRVX(mqttsn_client_init(&m_fg_mqttsn_client, MQTTSN_DEFAULT_CLIENT_PORT, fg_mqttsn_evt_handler,
        m_p_fg_mqttsn_thread_instance));
    fg_mqttsn_connect_opt_init();
}

FG_ACTOR_TASK_CALLBACK(fg_mqttsn_connect_cb)
{
    ASSERT(m_fg_actor_state == MQTTSN_KEEP_CONNECTED)
    NRFX_LOG_INFO("MQTTSN actor is connected");
}

static ret_code_t fg_mqttsn_progress();

FG_ACTOR_SLOT(fg_mqttsn_publish)
{
    if (m_fg_actor_state == MQTTSN_KEEP_ASLEEP)
    {
        FG_ACTOR_STATE_TRANSITION(
            MQTTSN_KEEP_ASLEEP, MQTTSN_KEEP_CONNECTED, "waking up and publishing MQTTSN message");
    }
    else
    {
        FG_ACTOR_STATE_TRANSITION(
            MQTTSN_KEEP_CONNECTED, MQTTSN_KEEP_CONNECTED, "publishing MQTTSN message");
    }

    if (m_fg_num_concurrent_messages < FG_MQTTSN_MAX_CONCURRENT_MESSAGES)
    {
        m_fg_num_concurrent_messages++;
        ASSERT(m_fg_num_concurrent_messages <= FG_MQTTSN_MAX_CONCURRENT_MESSAGES)

        FG_ACTOR_GET_P_ARGS(fg_mqttsn_message_t, p_mqttsn_message, p_calling_action);
        DRVX(nrf_atfifo_alloc_put(
            m_p_fg_mqttsn_message_fifo, &p_mqttsn_message, sizeof(p_mqttsn_message), NULL));
    }
    else
    {
        NRFX_LOG_ERROR("Message buffer is full, dropped message.");
        FG_ACTOR_ERROR(p_calling_action, NRFX_ERROR_NO_MEM);
    }

    DRVX(fg_mqttsn_progress());
}

FG_ACTOR_SLOT(fg_mqttsn_sleep)
{
    FG_ACTOR_STATE_TRANSITION(
        MQTTSN_KEEP_CONNECTED, MQTTSN_KEEP_ASLEEP, "putting MQTTSN client to sleep");

    // Get the sleep period (in ms) from the message.
    FG_ACTOR_GET_ARGS(uint32_t, sleep_poll_period_ms, p_calling_action); // in ms.

    // Make sure the sleep period is within valid bounds.
    ASSERT(sleep_poll_period_ms > FG_MQTT_THREAD_AWAKE_POLL_PERIOD &&
           sleep_poll_period_ms <= (FG_MQTT_THREAD_CHILD_TIMEOUT + 1) * 1000)
    m_fg_requested_sleep_poll_period = sleep_poll_period_ms;

    FG_ACTOR_RUN_SINGLETON_TASK(fg_mqttsn_sleep_cb);

    DRVX(fg_mqttsn_progress());
}

FG_ACTOR_TASK_CALLBACK(fg_mqttsn_sleep_cb)
{
    ASSERT(m_fg_actor_state == MQTTSN_KEEP_ASLEEP)
    NRFX_LOG_INFO("MQTTSN client is asleep");
}

FG_ACTOR_SLOT(fg_mqttsn_disconnect)
{
    FG_ACTOR_STATE_TRANSITION(
        MQTTSN_KEEP_CONNECTED, MQTTSN_KEEP_DISCONNECTED, "disconnecting MQTTSN client");

    FG_ACTOR_RUN_SINGLETON_TASK(fg_mqttsn_disconnect_cb);

    DRVX(fg_mqttsn_progress());
}

static ret_code_t fg_mqttsn_message_state_reset()
{
    m_fg_num_concurrent_messages = 0;
    return nrf_atfifo_clear(m_p_fg_mqttsn_message_fifo);
}

static void fg_mqttsn_gateway_state_reset()
{
    m_fg_mqttsn_gateway_addr = (mqttsn_remote_t){};
    m_fg_mqttsn_gateway_id = 0;
    for (int topic = 0; topic < FG_MQTT_TOPIC_NUM; topic++)
    {
        m_fg_mqttsn_topics[topic].topic_id = 0;
    }
}

FG_ACTOR_TASK_CALLBACK(fg_mqttsn_disconnect_cb)
{
    ASSERT(m_fg_actor_state == MQTTSN_KEEP_DISCONNECTED)

    DRVX(fg_mqttsn_message_state_reset());
    fg_mqttsn_gateway_state_reset();

    // Reset MQTT client.
    DRVX(mqttsn_client_uninit(&m_fg_mqttsn_client));
    m_fg_mqttsn_client.client_state = MQTTSN_CLIENT_UNINITIALIZED;

    // Reset MQTT transport.
    thread_deinit();
    m_p_fg_mqttsn_thread_instance = NULL;

    NRFX_LOG_INFO("MQTTSN client is disconnected");
}

FG_ACTOR_INTERFACE_DEF(mqttsn, fg_mqttsn_actor_message_code_t)

typedef enum
{
    FG_MQTTSN_EVENT_TYPE_TRANSPORT,
    FG_MQTTSN_EVENT_TYPE_CLIENT
} fg_mqttsn_event_type_t;

typedef enum
{
    FG_MQTTSN_TRANSPORT_EVENT_DETACHED,
    FG_MQTTSN_TRANSPORT_EVENT_ATTACHED
} fg_mqttsn_transport_event_t;

typedef struct
{
    const fg_mqttsn_event_type_t type;
    union
    { // type specific fields
        fg_mqttsn_transport_event_t mqtt_transport_event;
        mqttsn_event_t * mqtt_client_event;
    };
} fg_mqttsn_event_t;

typedef enum
{
    FG_MQTTSN_ACTIVITY_NONE,
    FG_MQTTSN_ACTIVITY_SEARCHING_GATEWAY,
    FG_MQTTSN_ACTIVITY_CONNECTING,
    FG_MQTTSN_ACTIVITY_REGISTERING_TOPIC,
    FG_MQTTSN_ACTIVITY_PUBLISHING,
    FG_MQTTSN_ACTIVITY_DISCONNECTING
} fg_mqttsn_activity_t;

static fg_mqttsn_activity_t m_fg_mqttsn_current_activity = FG_MQTTSN_ACTIVITY_NONE;

static bool fg_mqttsn_is_busy() { return m_fg_mqttsn_current_activity != FG_MQTTSN_ACTIVITY_NONE; }

static void fg_mqttsn_finish_activity(fg_mqttsn_activity_t finished_activity)
{
    if (m_fg_mqttsn_current_activity == finished_activity)
        m_fg_mqttsn_current_activity = FG_MQTTSN_ACTIVITY_NONE;
}

static void fg_mqttsn_start_activity(fg_mqttsn_activity_t started_activity)
{
    ASSERT(!fg_mqttsn_is_busy())
    m_fg_mqttsn_current_activity = started_activity;
}

static void fg_mqttsn_thread_sleep()
{
    ASSERT(m_fg_requested_sleep_poll_period > FG_MQTT_THREAD_AWAKE_POLL_PERIOD &&
           m_fg_requested_sleep_poll_period <= (FG_MQTT_THREAD_CHILD_TIMEOUT + 1) * 1000)
    otError error = otLinkSetPollPeriod(thread_ot_instance_get(), m_fg_requested_sleep_poll_period);
    ASSERT(error == OT_ERROR_NONE)
}

static void fg_mqttsn_thread_wake_up()
{
    otError error = otLinkSetPollPeriod(thread_ot_instance_get(), FG_MQTT_THREAD_AWAKE_POLL_PERIOD);
    ASSERT(error == OT_ERROR_NONE)
}

static uint8_t m_fg_mqttsn_num_lost_packages = 0;

static void fg_mqttsn_package_noack(mqttsn_error_t error)
{
    m_fg_mqttsn_num_lost_packages++;
    if (m_fg_mqttsn_num_lost_packages >= FG_MQTTSN_MAX_FAILED_PUBACK_BEFORE_RECONNECT ||
        error == MQTTSN_ERROR_REJECTED_INVALID_TOPIC)
    {
        m_fg_mqttsn_num_lost_packages = 0;
        fg_mqttsn_gateway_state_reset();
    }
}

static void fg_mqttsn_package_ack()
{
    m_fg_mqttsn_num_lost_packages = 0;
}

static void fg_mqttsn_update_state(fg_mqttsn_event_t * p_event)
{

    switch (p_event->type)
    {
        case FG_MQTTSN_EVENT_TYPE_TRANSPORT:
            switch (p_event->mqtt_transport_event)
            {
                case FG_MQTTSN_TRANSPORT_EVENT_DETACHED:
                    NRFX_LOG_INFO("MQTTSN transport event: detached.");
                    break;

                case FG_MQTTSN_TRANSPORT_EVENT_ATTACHED:
                    NRFX_LOG_INFO("MQTTSN transport event: attached.");
                    break;

                default:
                    NRFX_LOG_ERROR("MQTTSN event handler: Invalid transport event.");
                    ASSERT(false)
            }
            break;

        case FG_MQTTSN_EVENT_TYPE_CLIENT:
            switch (p_event->mqtt_client_event->event_id)
            {
                case MQTTSN_EVENT_CONNECTED:
                    NRFX_LOG_INFO("MQTTSN client event: Connected.");
                    fg_mqttsn_finish_activity(FG_MQTTSN_ACTIVITY_CONNECTING);
                    break;

                case MQTTSN_EVENT_DISCONNECTED:
                    NRFX_LOG_INFO("MQTTSN client event: Disconnected by gateway.");
                    break;

                case MQTTSN_EVENT_DISCONNECT_PERMIT:
                    NRFX_LOG_INFO("MQTTSN client event: Disconnect acknowledged.");
                    fg_mqttsn_finish_activity(FG_MQTTSN_ACTIVITY_DISCONNECTING);
                    break;

                case MQTTSN_EVENT_GATEWAY_FOUND:
                    NRFX_LOG_INFO("MQTTSN client event: Gateway found.");
                    m_fg_mqttsn_gateway_addr =
                        *(p_event->mqtt_client_event->event_data.connected.p_gateway_addr);
                    m_fg_mqttsn_gateway_id =
                        p_event->mqtt_client_event->event_data.connected.gateway_id;
                    break;

                case MQTTSN_EVENT_PUBLISHED:
                    NRFX_LOG_INFO("MQTTSN client event: Message published.");
                    fg_mqttsn_finish_activity(FG_MQTTSN_ACTIVITY_PUBLISHING);
                    fg_mqttsn_package_ack();
                    break;

                case MQTTSN_EVENT_REGISTERED:
                {
                    const mqttsn_topic_t registered_topic =
                        p_event->mqtt_client_event->event_data.registered.packet.topic;
                    NRFX_LOG_INFO("MQTTSN client event: Topic '%s' registered with ID: %d.",
                        registered_topic.p_topic_name, registered_topic.topic_id);
                    for (uint8_t topic = 0; topic < FG_MQTT_TOPIC_NUM; topic++)
                    {
                        if (strcmp(m_fg_mqttsn_topics[topic].p_topic_name,
                                registered_topic.p_topic_name) == 0)
                        {
                            m_fg_mqttsn_topics[topic].topic_id = registered_topic.topic_id;
                            break;
                        }
                    }
                    fg_mqttsn_finish_activity(FG_MQTTSN_ACTIVITY_REGISTERING_TOPIC);
                    fg_mqttsn_package_ack();
                    break;
                }

                case MQTTSN_EVENT_SLEEP_PERMIT:
                    NRFX_LOG_DEBUG("MQTTSN client event: Sleep permit.");
                    fg_mqttsn_thread_sleep();
                    fg_mqttsn_finish_activity(FG_MQTTSN_ACTIVITY_DISCONNECTING);
                    break;

                case MQTTSN_EVENT_SLEEP_STOP:
                    NRFX_LOG_DEBUG("MQTTSN client event: Awake for ping.");
                    fg_mqttsn_thread_wake_up();
                    break;

                case MQTTSN_EVENT_SEARCHGW_TIMEOUT:
                    if (p_event->mqtt_client_event->event_data.discovery ==
                        MQTTSN_SEARCH_GATEWAY_FINISHED)
                    {
                        NRFX_LOG_DEBUG("MQTTSN client event: Gateway discovery finished.");
                    }
                    else
                    {
                        NRFX_LOG_ERROR("MQTTSN client event: Gateway discovery error: 0x%x.",
                            p_event->mqtt_client_event->event_data.discovery);
                    }
                    fg_mqttsn_finish_activity(FG_MQTTSN_ACTIVITY_SEARCHING_GATEWAY);
                    break;

                case MQTTSN_EVENT_TIMEOUT:
                    switch (p_event->mqtt_client_event->event_data.error.msg_type)
                    {
                        case MQTTSN_PACKET_CONNACK:
                            NRFX_LOG_ERROR(
                                "MQTTSN client event: Connection failed: Error Type: %d.",
                                p_event->mqtt_client_event->event_data.error.error);
                            fg_mqttsn_finish_activity(FG_MQTTSN_ACTIVITY_CONNECTING);
                            break;

                        case MQTTSN_PACKET_REGACK:
                            NRFX_LOG_ERROR(
                                "MQTTSN client event: Topic registration failed: Error Type: %d.",
                                p_event->mqtt_client_event->event_data.error.error);
                            fg_mqttsn_finish_activity(FG_MQTTSN_ACTIVITY_REGISTERING_TOPIC);
                            fg_mqttsn_package_noack(p_event->mqtt_client_event->event_data.error.error);
                            break;

                        case MQTTSN_PACKET_PUBACK:
                            NRFX_LOG_ERROR(
                                "MQTTSN client event: Message publication failed: Error Type: %d.",
                                p_event->mqtt_client_event->event_data.error.error);
                            fg_mqttsn_finish_activity(FG_MQTTSN_ACTIVITY_PUBLISHING);
                            fg_mqttsn_package_noack(p_event->mqtt_client_event->event_data.error.error);
                            break;

                        case MQTTSN_PACKET_DISCONNECT:
                            NRFX_LOG_ERROR(
                                "MQTTSN client event: Disconnection failed: Error Type: %d.",
                                p_event->mqtt_client_event->event_data.error.error);
                                fg_mqttsn_finish_activity(FG_MQTTSN_ACTIVITY_DISCONNECTING);
                            break;

                        default:
                            NRFX_LOG_ERROR("MQTTSN client event: Packet rejected or timed out: "
                                           "Error Type: %d - "
                                           "Package Type: %d.",
                                p_event->mqtt_client_event->event_data.error.error,
                                p_event->mqtt_client_event->event_data.error.msg_type);
                    }
                    break;

                default:
                    NRFX_LOG_ERROR("MQTTSN client event: Unexpected event %d.",
                        p_event->mqtt_client_event->event_id);
            }
            break;

        default:
            NRFX_LOG_ERROR("MQTTSN event handler: Invalid event type.");
            ASSERT(false)
    }
}

static bool fg_mqttsn_is_transport_available()
{
    if (!m_p_fg_mqttsn_thread_instance)
        return false;

    otDeviceRole role = otThreadGetDeviceRole(m_p_fg_mqttsn_thread_instance);
    if (role >= OT_DEVICE_ROLE_CHILD)
        return true;

    return false;
}

static bool fg_mqttsn_is_gateway_known()
{
    return m_fg_mqttsn_gateway_addr.addr != 0 && m_fg_mqttsn_gateway_addr.port_number != 0 &&
           m_fg_mqttsn_gateway_id != 0;
}

static bool fg_mqttsn_is_asleep()
{
    mqttsn_client_state_t current_mqtt_state = mqttsn_client_state_get(&m_fg_mqttsn_client);
    return current_mqtt_state == MQTTSN_CLIENT_ASLEEP || current_mqtt_state == MQTTSN_CLIENT_AWAKE;
}

static bool fg_mqttsn_is_disconnected()
{
    mqttsn_client_state_t current_mqtt_state = mqttsn_client_state_get(&m_fg_mqttsn_client);
    return current_mqtt_state == MQTTSN_CLIENT_DISCONNECTED;
}

static bool fg_mqttsn_finish_task_if_running(fg_actor_task_callback_t cb)
{
    if (FG_ACTOR_IS_SINGLETON_TASK_RUNNING())
    {
        fg_actor_action_t * p_running_task = FG_ACTOR_GET_SINGLETON_TASK();
        if (p_running_task->task.callback == cb)
        {
            FG_ACTOR_SINGLETON_TASK_FINISHED();
            return true;
        }
    }
    return false;
}

static ret_code_t fg_schedule_next_queued_message()
{
    ASSERT(m_fg_num_concurrent_messages > 0)
    ASSERT(m_fg_actor_state == MQTTSN_KEEP_CONNECTED || m_fg_actor_state == MQTTSN_KEEP_ASLEEP)

    ret_code_t err_code;
    const fg_mqttsn_message_t * p_mqttsn_message;
    err_code = nrf_atfifo_get_free(
        m_p_fg_mqttsn_message_fifo, &p_mqttsn_message, sizeof(p_mqttsn_message), NULL);
    if (err_code != NRFX_SUCCESS)
    {
        NRF_LOG_ERROR("Error while trying to retrieve queued message from FIFO.");
        return err_code;
    }

    m_fg_num_concurrent_messages--;
    err_code = mqttsn_client_publish(&m_fg_mqttsn_client,
        m_fg_mqttsn_topics[p_mqttsn_message->topic_id].topic_id, p_mqttsn_message->p_data,
        p_mqttsn_message->size, &m_fg_mqttsn_msg_id);
    if (err_code == NRFX_SUCCESS)
        fg_mqttsn_start_activity(FG_MQTTSN_ACTIVITY_PUBLISHING);
    else
    {
        NRF_LOG_ERROR("Error while trying to schedule the next queued message.");
        return err_code;
    }

    return NRFX_SUCCESS;
}

#define FG_MQTTSN_RETCODE_IS_CONNECTED 999

static ret_code_t fg_mqttsn_ensure_connected()
{
    ret_code_t err_code;

    if (otLinkGetPollPeriod(thread_ot_instance_get()) != FG_MQTT_THREAD_AWAKE_POLL_PERIOD)
    {
        fg_mqttsn_thread_wake_up();
    }

    if (!fg_mqttsn_is_gateway_known())
    {
        if (!fg_mqttsn_is_disconnected())
        {
            err_code = mqttsn_client_disconnect(&m_fg_mqttsn_client);
            if (err_code == NRFX_SUCCESS)
                fg_mqttsn_start_activity(FG_MQTTSN_ACTIVITY_DISCONNECTING);
            return err_code;
        }

        err_code =
            mqttsn_client_search_gateway(&m_fg_mqttsn_client, FG_MQTT_SEARCH_GATEWAY_TIMEOUT);
        if (err_code == NRFX_SUCCESS)
            fg_mqttsn_start_activity(FG_MQTTSN_ACTIVITY_SEARCHING_GATEWAY);
        return err_code;
    }

    if (fg_mqttsn_is_disconnected() || fg_mqttsn_is_asleep())
    {
        err_code = mqttsn_client_connect(&m_fg_mqttsn_client, &m_fg_mqttsn_gateway_addr,
            m_fg_mqttsn_gateway_id, &m_fg_mqttsn_connect_opt);
        if (err_code == NRFX_SUCCESS)
            fg_mqttsn_start_activity(FG_MQTTSN_ACTIVITY_CONNECTING);
        return err_code;
    }

    return FG_MQTTSN_RETCODE_IS_CONNECTED;
}

static ret_code_t fg_mqttsn_progress()
{
    if (!fg_mqttsn_is_transport_available() || fg_mqttsn_is_busy())
        return NRFX_SUCCESS;

    ret_code_t err_code;

    bool has_pending_message = m_fg_num_concurrent_messages > 0;
    if (has_pending_message || m_fg_actor_state == MQTTSN_KEEP_CONNECTED)
    {
        err_code = fg_mqttsn_ensure_connected();
        if (err_code != FG_MQTTSN_RETCODE_IS_CONNECTED)
            return err_code;

        for (uint8_t topic = 0; topic < FG_MQTT_TOPIC_NUM; topic++)
        {
            if (m_fg_mqttsn_topics[topic].topic_id == 0)
            {
                err_code = mqttsn_client_topic_register(&m_fg_mqttsn_client,
                    m_fg_mqttsn_topics[topic].p_topic_name,
                    strlen(m_fg_mqttsn_topics[topic].p_topic_name), &m_fg_mqttsn_msg_id);
                if (err_code == NRFX_SUCCESS)
                    fg_mqttsn_start_activity(FG_MQTTSN_ACTIVITY_REGISTERING_TOPIC);
                return err_code;
            }
        }

        if (m_fg_actor_state == MQTTSN_KEEP_CONNECTED)
            fg_mqttsn_finish_task_if_running(fg_mqttsn_connect_cb);

        if (has_pending_message)
            return fg_schedule_next_queued_message();
    }

    if (m_fg_actor_state == MQTTSN_KEEP_ASLEEP)
    {
        if (fg_mqttsn_is_asleep())
        {
            fg_mqttsn_finish_task_if_running(fg_mqttsn_sleep_cb);
            return NRFX_SUCCESS;
        }

        err_code = fg_mqttsn_ensure_connected();
        if (err_code != FG_MQTTSN_RETCODE_IS_CONNECTED)
            return err_code;

        err_code = mqttsn_client_sleep(&m_fg_mqttsn_client, FG_MQTT_THREAD_SLEEP_TIMEOUT);
        if (err_code == NRFX_SUCCESS)
            fg_mqttsn_start_activity(FG_MQTTSN_ACTIVITY_DISCONNECTING);
        return err_code;
    }

    if (m_fg_actor_state == MQTTSN_KEEP_DISCONNECTED)
    {
        if (fg_mqttsn_is_disconnected())
        {
            fg_mqttsn_finish_task_if_running(fg_mqttsn_disconnect_cb);
            return NRFX_SUCCESS;
        }

        err_code = mqttsn_client_disconnect(&m_fg_mqttsn_client);
        if (err_code == NRFX_SUCCESS)
            fg_mqttsn_start_activity(FG_MQTTSN_ACTIVITY_DISCONNECTING);
        return err_code;
    }

    return NRFX_SUCCESS;
}

static void fg_mqttsn_event_handler_cb(fg_mqttsn_event_t * p_event)
{
    fg_mqttsn_update_state(p_event);

    APP_ERROR_CHECK(fg_mqttsn_progress());
}

static void fg_mqttsn_thread_state_changed_cb(uint32_t flags, void * p_context)
{
    if (!flags & OT_CHANGED_THREAD_ROLE)
        return;

    const bool is_transport_available = fg_mqttsn_is_transport_available();
    if (is_transport_available)
    {
        NRFX_LOG_DEBUG("Thread network attached.");
    }
    else
    {
        NRFX_LOG_DEBUG("Thread network detached.");
    }

    static fg_mqttsn_event_t transport_event = {.type = FG_MQTTSN_EVENT_TYPE_TRANSPORT};
    transport_event.mqtt_transport_event = is_transport_available
                                               ? FG_MQTTSN_TRANSPORT_EVENT_ATTACHED
                                               : FG_MQTTSN_TRANSPORT_EVENT_DETACHED;
    fg_mqttsn_event_handler_cb(&transport_event);
}

static void fg_mqttsn_evt_handler(mqttsn_client_t * p_client, mqttsn_event_t * p_event)
{
    static fg_mqttsn_event_t client_event = {.type = FG_MQTTSN_EVENT_TYPE_CLIENT};
    client_event.mqtt_client_event = p_event;
    fg_mqttsn_event_handler_cb(&client_event);
}