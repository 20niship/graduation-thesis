#pragma once
#include <array>
#include <iostream>

#include <modbus/modbus.h>

#include <hr4c/ModbusClient.hpp>

// from https://github.com/stephane/libmodbus/blob/master/tests/unit-test.h.in
#define SERVER_ID 17
#define INVALID_SERVER_ID 18

const uint16_t UT_BITS_ADDRESS = 0x130;
const uint16_t UT_BITS_NB      = 0x25;
const uint8_t UT_BITS_TAB[]    = {0xCD, 0x6B, 0xB2, 0x0E, 0x1B};

const uint16_t UT_INPUT_BITS_ADDRESS = 0x1C4;
const uint16_t UT_INPUT_BITS_NB      = 0x16;
const uint8_t UT_INPUT_BITS_TAB[]    = {0xAC, 0xDB, 0x35};

const uint16_t UT_REGISTERS_ADDRESS = 0x160;
const uint16_t UT_REGISTERS_NB      = 0x3;
const uint16_t UT_REGISTERS_NB_MAX  = 0x20;
const uint16_t UT_REGISTERS_TAB[]   = {0x022B, 0x0001, 0x0064};

/* Raise a manual exception when this address is used for the first byte */
const uint16_t UT_REGISTERS_ADDRESS_SPECIAL = 0x170;
/* The response of the server will contains an invalid TID or slave */
const uint16_t UT_REGISTERS_ADDRESS_INVALID_TID_OR_SLAVE = 0x171;
/* The server will wait for 1 second before replying to test timeout */
const uint16_t UT_REGISTERS_ADDRESS_SLEEP_500_MS = 0x172;
/* The server will wait for 5 ms before sending each byte */
const uint16_t UT_REGISTERS_ADDRESS_BYTE_SLEEP_5_MS = 0x173;

/* If the following value is used, a bad response is sent.
   It's better to test with a lower value than
   UT_REGISTERS_NB_POINTS to try to raise a segfault. */
const uint16_t UT_REGISTERS_NB_SPECIAL = 0x2;

const uint16_t UT_INPUT_REGISTERS_ADDRESS = 0x108;
const uint16_t UT_INPUT_REGISTERS_NB      = 0x1;
const uint16_t UT_INPUT_REGISTERS_TAB[]   = {0x000A};

/*
 * This float value is 0x47F12000 (in big-endian format).
 * In Little-endian(intel) format, it will be stored in memory as follows:
 * 0x00 0x20 0xF1 0x47
 *
 * You can check this with the following code:

   float fl = UT_REAL;
   uint8_t *inmem = (uint8_t*)&fl;
   int x;
   for(x = 0; x < 4; x++){
       printf("0x%02X ", inmem[ x ]);
   }
   printf("\n");
 */
const float UT_REAL = 123456.00;

/*
 * The following arrays assume that 'A' is the MSB,
 * and 'D' is the LSB.
 * Thus, the following is the case:
 * A = 0x47
 * B = 0xF1
 * C = 0x20
 * D = 0x00
 *
 * There are two sets of arrays: one to test that the setting is correct,
 * the other to test that the getting is correct.
 * Note that the 'get' values must be constants in processor-endianness,
 * as libmodbus will convert all words to processor-endianness as they come in.
 */
const uint8_t UT_IREAL_ABCD_SET[]  = {0x47, 0xF1, 0x20, 0x00};
const uint16_t UT_IREAL_ABCD_GET[] = {0x47F1, 0x2000};
const uint8_t UT_IREAL_DCBA_SET[]  = {0x00, 0x20, 0xF1, 0x47};
const uint16_t UT_IREAL_DCBA_GET[] = {0x0020, 0xF147};
const uint8_t UT_IREAL_BADC_SET[]  = {0xF1, 0x47, 0x00, 0x20};
const uint16_t UT_IREAL_BADC_GET[] = {0xF147, 0x0020};
const uint8_t UT_IREAL_CDAB_SET[]  = {0x20, 0x00, 0x47, 0xF1};
const uint16_t UT_IREAL_CDAB_GET[] = {0x2000, 0x47F1};

/* const uint32_t UT_IREAL_ABCD = 0x47F12000);
const uint32_t UT_IREAL_DCBA = 0x0020F147;
const uint32_t UT_IREAL_BADC = 0xF1470020;
const uint32_t UT_IREAL_CDAB = 0x200047F1;*/


namespace hr4c {

class ModbusServer {
  ModbusServer()  = default;
  ~ModbusServer() = default;

private:
  modbus_t* m_ctx;
  std::array<uint16_t, MODBUS_AXIS_DATA_NUM> m_tab_reg;
  std::array<uint16_t, MODBUS_COMMAND_DATA_NUM> m_cmd_reg;
  bool m_connected = false;

  int header_length = 0;
  modbus_mapping_t* mb_mapping;

public:
  bool start(int port) {
    std::string ip_or_device = "127.0.0.1";

    m_ctx         = modbus_new_tcp(ip_or_device.c_str(), port);
    header_length = modbus_get_header_length(m_ctx);

    modbus_set_debug(m_ctx, TRUE);
    mb_mapping = modbus_mapping_new_start_address(UT_BITS_ADDRESS, UT_BITS_NB, UT_INPUT_BITS_ADDRESS, UT_INPUT_BITS_NB, UT_REGISTERS_ADDRESS, UT_REGISTERS_NB_MAX, UT_INPUT_REGISTERS_ADDRESS, UT_INPUT_REGISTERS_NB);
    if(mb_mapping == NULL) {
      fprintf(stderr, "Failed to allocate the mapping: %s\n", modbus_strerror(errno));
      modbus_free(m_ctx);
      return -1;
    }

    /* Examples from PI_MODBUS_300.pdf.
       Only the read-only input values are assigned. */

    /* Initialize input values that's can be only done server side. */
    modbus_set_bits_from_bytes(mb_mapping->tab_input_bits, 0, UT_INPUT_BITS_NB, UT_INPUT_BITS_TAB);

    /* Initialize values of INPUT REGISTERS */
    for(int i = 0; i < UT_INPUT_REGISTERS_NB; i++) {
      mb_mapping->tab_input_registers[i] = UT_INPUT_REGISTERS_TAB[i];
    }

    int s = modbus_tcp_listen(m_ctx, 1);
    modbus_tcp_accept(m_ctx, &s);

    return true;
  }

  bool connect();

  bool send_axis_data() const;
  bool send_command_data() const;
  bool read_axis_data();
  bool read_command_data();

  template <typename T> T read_axis_data(int index) const {
    assert(index < MODBUS_AXIS_DATA_NUM);
    return static_cast<T>(m_tab_reg[index]);
  }
  template <typename T> T read_command_data(int index) const {
    assert(index < MODBUS_COMMAND_DATA_NUM);
    return static_cast<T>(m_cmd_reg[index]);
  }

  template <typename T> void set_axis_data(int index, const T data) {
    assert(index < MODBUS_AXIS_DATA_NUM);
    const void* data_ptr = static_cast<const void*>(&data);
    for(int i = 0; i < sizeof(T) / sizeof(uint16_t); i++) {
      m_tab_reg[index + i] = *(static_cast<const uint16_t*>(data_ptr) + i);
    }
  }
  template <typename T> void set_command_data(int index, const T data) {
    assert(index < MODBUS_AXIS_DATA_NUM);
    const void* data_ptr = static_cast<const void*>(&data);
    for(int i = 0; i < sizeof(T) / sizeof(uint16_t); i++) {
      m_cmd_reg[index + i] = *(static_cast<const uint16_t*>(data_ptr) + i);
    }
  }

  bool disconnect();


  void test() {
    for(;;) {
      int rc;
      do {
        rc = modbus_receive(m_ctx, (uint8_t*)m_tab_reg.data());
        /* Filtered queries return 0 */
      } while(rc == 0);

      /* The connection is not closed on errors which require on reply such as
         bad CRC in RTU. */
      if(rc == -1 && errno != EMBBADCRC) {
        /* Quit */
        break;
      }

      /* Special server behavior to test client */
      if(m_tab_reg[header_length] == 0x03) {
        /* Read holding registers */

        if(MODBUS_GET_INT16_FROM_INT8(m_tab_reg, header_length + 3) == UT_REGISTERS_NB_SPECIAL) {
          printf("Set an incorrect number of values\n");
          MODBUS_SET_INT16_TO_INT8(m_tab_reg.data(), header_length + 3, UT_REGISTERS_NB_SPECIAL - 1);
        } else if(MODBUS_GET_INT16_FROM_INT8(m_tab_reg, header_length + 1) == UT_REGISTERS_ADDRESS_SPECIAL) {
          printf("Reply to this special register address by an exception\n");
          modbus_reply_exception(m_ctx, (uint8_t*)m_tab_reg.data(), MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY);
          continue;
        } else if(MODBUS_GET_INT16_FROM_INT8(m_tab_reg, header_length + 1) == UT_REGISTERS_ADDRESS_INVALID_TID_OR_SLAVE) {
          const int RAW_REQ_LENGTH = 5;
          uint8_t raw_req[]        = {(use_backend == RTU) ? INVALID_SERVER_ID : 0xFF, 0x03, 0x02, 0x00, 0x00};

          printf("Reply with an invalid TID or slave\n");
          modbus_send_raw_request(m_ctx, raw_req, RAW_REQ_LENGTH * sizeof(uint8_t));
          continue;
        } else if(MODBUS_GET_INT16_FROM_INT8(m_tab_reg, header_length + 1) == UT_REGISTERS_ADDRESS_SLEEP_500_MS) {
          printf("Sleep 0.5 s before replying\n");
          usleep(500000);
        } else if(MODBUS_GET_INT16_FROM_INT8(m_tab_reg, header_length + 1) == UT_REGISTERS_ADDRESS_BYTE_SLEEP_5_MS) {
          /* Test low level only available in TCP mode */
          /* Catch the reply and send reply byte a byte */
          uint8_t req[]  = "\x00\x1C\x00\x00\x00\x05\xFF\x03\x02\x00\x00";
          int req_length = 11;
          int w_s        = modbus_get_socket(m_ctx);
          if(w_s == -1) {
            fprintf(stderr, "Unable to get a valid socket in special test\n");
            continue;
          }

          /* Copy TID */
          req[1] = m_tab_reg[1];
          for(i = 0; i < req_length; i++) {
            printf("(%.2X)", req[i]);
            usleep(5000);
            rc = send(w_s, (const char*)(req + i), 1, MSG_NOSIGNAL);
            if(rc == -1) {
              break;
            }
          }
          continue;
        }
      }

      rc = modbus_reply(m_ctx, m_tab_reg, rc, mb_mapping);
      if(rc == -1) {
        break;
      }
    }

    printf("Quit the loop: %s\n", modbus_strerror(errno));

    if(use_backend == TCP) {
      if(s != -1) {
        close(s);
      }
    }
    modbus_mapping_free(mb_mapping);
    free(m_tab_reg);
    /* For RTU */
    modbus_close(m_ctx);
    modbus_free(m_ctx);

    return 0;
  }
}
};


} // namespace hr4c
