#include <WiFi.h>
#include <PubSubClient.h>

struct endl_t {} endl;

/* Settings */

#include "garage_mqtt_settings.h"

#ifndef GARAGE_MQTT_SETTINGS_H
#error "You have not created a settings file."
#endif

#define TOPIC_PREFIX _TOPIC_PREFIX
#define STATE_TOPIC_SUFFIX _STATE_TOPIC_SUFFIX
#define CMD_TOPIC_SUFFIX _CMD_TOPIC_SUFFIX
#define CMD_TOPIC_SUFFIX _CMD_TOPIC_SUFFIX
#define AVAILABILITY_TOPIC_SUFFIX _AVAILABILITY_TOPIC_SUFFIX

const char WIFI_SSID[] = _WIFI_SSID;
const char WIFI_PASSPHRASE[] = _WIFI_PASSPHRASE;
constexpr unsigned short WIFI_STATUS_PIN = _WIFI_STATUS_PIN;

const char MQTT_HOST[] = _MQTT_HOST;
const uint16_t MQTT_PORT = _MQTT_PORT;
const char MQTT_USER[] = _MQTT_USER;
const char MQTT_PASSWORD[] = _MQTT_PASSWORD;
const char MQTT_CLIENT_ID[] = _MQTT_CLIENT_ID;

constexpr unsigned int AVAILABILITY_BROADCAST_DURATION = _AVAILABILITY_BROADCAST_DURATION;

/* Constants */

static const char STATE_OPEN[] = "open";
static const char STATE_CLOSED[] = "closed";
static const char STATE_OPENING[] = "opening";
static const char STATE_CLOSING[] = "closing";

static const char CMD_OPEN[] = "OPEN";
static const char CMD_CLOSE[] = "CLOSE";
static const char CMD_STOP[] = "STOP";

static const char AVAILABILITY_ONLINE[] = "online";
static const char AVAILABILITY_OFFLINE[] = "offline";

constexpr unsigned int INVALID_PIN = static_cast<unsigned int>(-1);

/* Hi. Did you know that Arduino coding is C++? This block is here to remind you of that! */

template<typename T>
constexpr T max_c(T a, T b)
{
  return a > b ? a : b;
}

template<typename T>
constexpr T min_c(T a, T b)
{
  return a < b ? a : b;
}

template<typename T>
inline Print& operator<<(Print& printer, T value)
{
  printer.print(value);
  return printer;
}

template<>
inline Print& operator<<(Print& printer, endl_t)
{
  printer.println();
  return printer;
}

template<>
inline Print& operator<<(Print& printer, const char* value)
{
  printer.print(value ? value : "(nullptr)");
  return printer;
}

constexpr unsigned int CMD_MAX_LEN = max_c(max_c(sizeof(CMD_OPEN), sizeof(CMD_CLOSE)), sizeof(CMD_STOP)) - 1;
constexpr unsigned int CMD_MIN_LEN = min_c(min_c(sizeof(CMD_OPEN), sizeof(CMD_CLOSE)), sizeof(CMD_STOP)) - 1;

/* Global State */

WiFiClient gWiFiClient;

/* Functions */

const char* payload_to_cmd(byte* payload, unsigned int len)
{
  if(len > CMD_MAX_LEN || len < CMD_MIN_LEN)
  {
    return nullptr;
  }

  if(len == sizeof(CMD_OPEN) - 1 && !strncmp(reinterpret_cast<const char*>(payload), CMD_OPEN, len))
  {
    return CMD_OPEN;
  }
  if(len == sizeof(CMD_CLOSE) - 1 && !strncmp(reinterpret_cast<const char*>(payload), CMD_CLOSE, len))
  {
    return CMD_CLOSE;
  }
  if(len == sizeof(CMD_STOP) - 1 && !strncmp(reinterpret_cast<const char*>(payload), CMD_STOP, len))
  {
    return CMD_STOP;
  }

  return nullptr;
}

void connect_wifi(bool do_setup=false)
{
  if(WIFI_STATUS_PIN && do_setup)
  {
    pinMode(WIFI_STATUS_PIN, OUTPUT);
  }

  if(do_setup || WiFi.status() != WL_CONNECTED)
  {
    if(WIFI_STATUS_PIN)
    {
      digitalWrite(WIFI_STATUS_PIN, LOW);
    }

    Serial << endl << endl;
    Serial << "Connecting to " << WIFI_SSID << endl;

    WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);

    while(WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial << ".";
    }

    Serial << " done!" << endl;
    Serial << "IP Address: " << WiFi.localIP() << endl;
  }

  if(WIFI_STATUS_PIN)
  {
    digitalWrite(WIFI_STATUS_PIN, HIGH);
  }
}

/* MQTT Logic */

class MQTTManager
{
public:
  typedef void(*callback_t)(void*, byte*, unsigned int);

  static void setup()
  {
    m_client.setServer(MQTT_HOST, MQTT_PORT);
    m_client.setCallback(_callback);

    MQTTManager* curr = m_head;
    while(curr)
    {
      curr->_setup();
      curr = curr->m_next;
    }

    _reconnect();
  }

  static void loop()
  {
    _reconnect();
    m_client.loop();

    MQTTManager* curr = m_head;
    while(curr)
    {
      curr->_loop();
      curr = curr->m_next;
    }
  }

  MQTTManager(const char* device_name, callback_t callback_func, void* user_data) :
    m_device_name(strdup(device_name)),
    m_next(m_head),
    m_callback(callback_func),
    m_user_data(user_data)
  {
    size_t device_name_len = strlen(device_name);
    m_state_topic = (char*)malloc(sizeof(TOPIC_PREFIX) + sizeof(STATE_TOPIC_SUFFIX) + device_name_len - 1);
    m_cmd_topic = (char*)malloc(sizeof(TOPIC_PREFIX) + sizeof(CMD_TOPIC_SUFFIX) + device_name_len - 1);
    m_availability_topic = (char*)malloc(sizeof(TOPIC_PREFIX) + sizeof(AVAILABILITY_TOPIC_SUFFIX) + device_name_len - 1);

    if(!m_device_name || !m_state_topic || !m_cmd_topic || !m_availability_topic)
    {
      return;
    }

    m_head = this;

    m_state_topic[0] = 0;
    strcat(m_state_topic, TOPIC_PREFIX);
    strcat(m_state_topic, device_name);
    strcat(m_state_topic, STATE_TOPIC_SUFFIX);

    m_cmd_topic[0] = 0;
    strcat(m_cmd_topic, TOPIC_PREFIX);
    strcat(m_cmd_topic, device_name);
    strcat(m_cmd_topic, CMD_TOPIC_SUFFIX);

    m_availability_topic[0] = 0;
    strcat(m_availability_topic, TOPIC_PREFIX);
    strcat(m_availability_topic, device_name);
    strcat(m_availability_topic, AVAILABILITY_TOPIC_SUFFIX);
  }

  void publish_state(const char* state)
  {
    m_client.publish(m_state_topic, state);
  }

  void publish_availability(bool online)
  {
    m_current_availability = online;
  }

  ~MQTTManager()
  {
    if(m_head == this)
    {
      m_head = m_next;
    }
    else
    {
      MQTTManager* curr = m_head;
      while(curr)
      {
        if(curr->m_next == this)
        {
          curr->m_next = m_next;
          break;
        }
        curr = curr->m_next;
      }
    }

    m_client.unsubscribe(m_cmd_topic);

    free(m_device_name);
    free(m_cmd_topic);
    free(m_state_topic);
    free(m_availability_topic);
  }

private:
  static PubSubClient m_client;
  static MQTTManager* m_head;

  char* m_state_topic = nullptr;
  char* m_cmd_topic = nullptr;
  char* m_availability_topic = nullptr;
  char* m_device_name;
  MQTTManager* m_next;
  callback_t m_callback;
  void* m_user_data;
  bool m_last_availability_broadcast = false;
  unsigned int m_last_availability_broadcast_time = 0;
  bool m_current_availability = false;

  void _setup()
  {
    Serial << "Setting up MQTT for " << m_device_name << endl;

    if(!m_device_name || !m_state_topic || !m_cmd_topic)
    {
      Serial << "MQTT: Out of memory trying to configure device " << m_device_name << endl;
      return;
    }

    if(!m_callback)
    {
      Serial << "MQTT: Warning: No callback specified for " << m_device_name << endl;
    }

    Serial << "MQTT Device:   " << m_device_name << endl;
    Serial << "Command Topic: " << m_cmd_topic << endl;
    Serial << "State Topic:   " << m_state_topic << endl;
  }

  void _loop()
  {
    unsigned int current_time = millis();
    if(current_time < m_last_availability_broadcast && current_time < AVAILABILITY_BROADCAST_DURATION) // millis() wrapped around
    {
      m_last_availability_broadcast = 0;
    }
    else if(m_last_availability_broadcast != m_current_availability || m_last_availability_broadcast + AVAILABILITY_BROADCAST_DURATION < current_time)
    {
      m_client.publish(m_availability_topic, m_current_availability ? AVAILABILITY_ONLINE : AVAILABILITY_OFFLINE);
      m_last_availability_broadcast = m_current_availability;
      m_last_availability_broadcast = m_last_availability_broadcast;
      Serial << "Availability is now: " << m_current_availability << endl;
    }
  }

  static void _callback(char* topic, byte* payload, unsigned int payload_len)
  {
    MQTTManager* curr = m_head;
    while(curr)
    {
      if(!strcmp(topic, curr->m_cmd_topic) && curr->m_callback)
      {
        curr->m_callback(curr->m_user_data, payload, payload_len);
        break;
      }
      curr = curr->m_next;
    }
  }

  static void _reconnect()
  {
    while(!m_client.connected())
    {
      Serial << "Attempting MQTT connection to " << MQTT_USER << '@' << MQTT_HOST << ':' << MQTT_PORT << "...";
      if (m_client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD))
      {
        Serial << " connected!" << endl;
        MQTTManager* curr = m_head;
        while(curr)
        {
          m_client.subscribe(curr->m_cmd_topic);
          curr = curr->m_next;
        }
      }
      else
      {
        Serial << " failed. state=" << m_client.state() << endl;
        delay(5000);
      }
    }
  }

};

PubSubClient MQTTManager::m_client(gWiFiClient);
MQTTManager* MQTTManager::m_head(nullptr);

/* Door Logic */

class DoorStateMachine
{
public:
  enum CommandResponse
  {
    OK,
    BUSY,
    INVALID
  };

 DoorStateMachine(const char* device_name, const unsigned int relay_pin, const unsigned int closed_pin, const unsigned int open_pin = INVALID_PIN, const unsigned int stop_relay_pin = INVALID_PIN) :
    m_mqtt_manager(device_name, mqtt_callback, this),
    m_relay_pin(relay_pin),
    m_closed_pin(closed_pin),
    m_open_pin(open_pin),
    m_stop_relay_pin(stop_relay_pin)
  {
  }

  void setup()
  {
    Serial << "Relay Pin:      " << m_relay_pin << endl;
    Serial << "Closed Pin:     " << m_closed_pin << endl;
    Serial << "Open Pin:       " << m_open_pin << endl;
    Serial << "Stop Relay Pin: " << m_stop_relay_pin << endl;

    pinMode(m_relay_pin, OUTPUT);
    digitalWrite(m_relay_pin, LOW);
    pinMode(m_closed_pin, INPUT_PULLUP);

    if(m_open_pin != INVALID_PIN)
    {
      pinMode(m_open_pin, INPUT_PULLUP);
    }

    m_mqtt_manager.publish_availability(true);
  }

  void loop()
  {
    if(millis() <= m_debounce_until)
    {
      return;
    }

    bool door_closed = !digitalRead(m_closed_pin);
    bool door_open = m_open_pin == INVALID_PIN ? !door_closed : !digitalRead(m_open_pin);

    const char* last_state = m_current_state;
    const char* last_command = m_issued_command;

    /* Manage States */
    if(door_open && door_closed)
    {
      Serial << "DoorStateMachine: GPIO is reporting that door is both open and closed. Check your sensors." << endl;
      m_mqtt_manager.publish_availability(false);
      delay(1000);
      return;
    }
    else
    {
      m_mqtt_manager.publish_availability(true);
    }

    if(door_open)
    {
      m_current_state = STATE_OPEN;
    }
    else if(door_closed)
    {
      m_current_state = STATE_CLOSED;
    }
    else if(m_current_state == STATE_CLOSED)
    {
      m_current_state = STATE_OPENING;
    }
    else if(m_current_state == STATE_OPEN)
    {
      m_current_state = STATE_CLOSING;
    }

    /* React To Commands */
    if(m_issued_command == CMD_OPEN)
    {
      if (m_current_state == STATE_CLOSED || m_current_state == STATE_CLOSING)
      {
        Serial << "DoorStateMachine: Open command dispatched." << endl;
        pulse_relay();
      }
      else if (m_current_state == STATE_OPEN || m_current_state == STATE_OPENING)
      {
        Serial << "DoorStateMachine: Open command ignored, door is already " << m_current_state << endl;
      }
      else
      {
        Serial << "DoorStateMachine: Door is in invalid state: " << reinterpret_cast<size_t>(m_current_state) << endl;
      }
    }
    else if(m_issued_command == CMD_CLOSE)
    {
      if (m_current_state == STATE_OPEN || m_current_state == STATE_OPENING)
      {
        Serial << "DoorStateMachine: Close command dispatched." << endl;
        pulse_relay();
      }
      else if (m_current_state == STATE_CLOSED || m_current_state == STATE_CLOSING)
      {
        Serial << "DoorStateMachine: Close command ignored, door is already " << m_current_state << endl;
      }
      else
      {
        Serial << "DoorStateMachine: Door is in invalid state: " << reinterpret_cast<size_t>(m_current_state) << endl;
      }
    }
    else if(m_issued_command == CMD_STOP)
    {
      if(m_stop_relay_pin != INVALID_PIN)
      {
        Serial << "DoorStateMachine: Stop command dispatched." << endl;
        pulse_stop_relay();
      }
      else if(m_current_state == STATE_CLOSING)
      {
        Serial << "DoorStateMachine: Stop command dispatched." << endl;
        pulse_relay();
      }
      else
      {
        Serial << "DoorStateMachine: Stop command ignored. No stop relay and the the door isn't closing." << endl;
      }
    }

    m_issued_command = nullptr;

    /* Debugging Output & GPIO Debounce */
    if(last_state != m_current_state)
    {
      m_debounce_until = millis() + 50;
      Serial << "DoorStateMachine: State transioned from " << last_state << " to " << m_current_state << " (door_open: " << door_open << ", door_closed: " << door_closed << ")" << endl;
      m_mqtt_manager.publish_state(m_current_state);
    }
    if(last_command && !m_issued_command)
    {
      Serial << "DoorStateMachine: Finished command " << last_command << endl;
    }
  }

  CommandResponse issue_command(const char* cmd)
  {
    if(m_issued_command)
    {
      return BUSY;
    }

    if(cmd == CMD_OPEN)
    {
      Serial << "DoorStateMachine: Open Requested" << endl;
      m_issued_command = CMD_OPEN;
    }
    else if(cmd == CMD_CLOSE)
    {
      Serial << "DoorStateMachine: Close Requested" << endl;
      m_issued_command = CMD_CLOSE;
    }
    else if(cmd == CMD_STOP)
    {
      Serial << "DoorStateMachine: Stop Requested" << endl;
      m_issued_command = CMD_STOP;
    }
    else
    {
      Serial << "DoorStateMachine: Invalid command issued." << endl;
      return INVALID;
    }
    return OK;
  }

  void pulse_relay()
  {
    Serial << "DoorStateMachine: Relay Pulsed (Pin: " << m_relay_pin << ")" << endl;
    digitalWrite(m_relay_pin, HIGH);
    delay(250);
    digitalWrite(m_relay_pin, LOW);
  }

  void pulse_stop_relay()
  {
    Serial << "DoorStateMachine: Stop Relay Pulsed (Pin: " << m_stop_relay_pin << ")" << endl;
    if(m_stop_relay_pin != INVALID_PIN)
    {
      digitalWrite(m_stop_relay_pin, HIGH);
      delay(250);
      digitalWrite(m_stop_relay_pin, LOW);
    }
    else
    {
      delay(250);
    }
  }

  static void mqtt_callback(void* this_ptr, byte* payload, unsigned int payload_len)
  {
    const char* cmd = payload_to_cmd(payload, payload_len);
    DoorStateMachine* self = reinterpret_cast<DoorStateMachine*>(this_ptr);

    if(cmd == CMD_OPEN)
    {
      Serial << "MQTT: Open Requested" << endl;
      self->issue_command(CMD_OPEN);
    }
    else if(cmd == CMD_CLOSE)
    {
      Serial << "MQTT: Close Requested" << endl;
      self->issue_command(CMD_CLOSE);
    }
    else if(cmd == CMD_STOP)
    {
      Serial << "MQTT: Stop Requested" << endl;
      self->issue_command(CMD_STOP);
    }
    else
    {
      Serial << "MQTT: Invalid Message (length=" << payload_len << ") from command topic: ";
      for(unsigned int i = 0; i < payload_len; ++i)
      {
        char c = payload[i];
        if(c < 32 || c > 126)
        {
          c = '.';
        }
        Serial << c;
      }
      Serial << endl;
    }
  }

private:
  const char* m_current_state = nullptr;
  const char* m_issued_command = nullptr;
  unsigned long m_debounce_until = 0ul;
  MQTTManager m_mqtt_manager;

  const unsigned int m_relay_pin;
  const unsigned int m_closed_pin;
  const unsigned int m_open_pin;
  const unsigned int m_stop_relay_pin;
};

DoorStateMachine gDoorStates[] = _DEFINITIONS;
constexpr unsigned int NUM_DOORS = sizeof(gDoorStates) / sizeof(gDoorStates[0]);

void setup()
{
  Serial.begin(115200);
  Serial << "Initializing..." << endl;

  connect_wifi(true);

  MQTTManager::setup();
  for(int i = 0; i < NUM_DOORS; ++i)
  {
    gDoorStates[i].setup();
  }

  Serial << "Initialization done." << endl;
}

void loop()
{
  connect_wifi();
  MQTTManager::loop();
  for(int i = 0; i < NUM_DOORS; ++i)
  {
    gDoorStates[i].loop();
  }
}
