#include "gtest/gtest.h"
#include "mraa/initio.h"
#include <iostream>

/* MRAA API common test fixture */
class mraa_initio_h_unit : public ::testing::Test
{
protected:
  /* One-time setup logic if needed */
  mraa_initio_h_unit() {}

  /* One-time tear-down logic if needed */
  virtual ~mraa_initio_h_unit() {}

  /* Per-test setup logic if needed */
  virtual void SetUp() {}

  /* Per-test tear-down logic if needed */
  virtual void TearDown() {}
};

/* Test for a successful AIO init. */
TEST_F(mraa_initio_h_unit, test_aio_init)
{
  /*mraa_io_descriptor* desc;
  mraa_result_t status;

  status = mraa_io_init("a:1:10", &desc);
  ASSERT_EQ(status, MRAA_SUCCESS);

  status = mraa_io_close(desc);
  ASSERT_EQ(status, MRAA_SUCCESS);*/
}

/* Test for a successful GPIO init. */
TEST_F(mraa_initio_h_unit, test_gpio_init)
{
  /*mraa_io_descriptor* desc;
  mraa_result_t status;

  status = mraa_io_init("g:1:1", &desc);
  ASSERT_EQ(status, MRAA_SUCCESS);

  status = mraa_io_close(desc);
  ASSERT_EQ(status, MRAA_SUCCESS);*/
}

/* Test for a successful I2C init. */
TEST_F(mraa_initio_h_unit, test_i2c_init)
{
  /*mraa_io_descriptor* desc;
   m raa_result_t status;

   status = mraa_io_init("i:1:16", &desc);
   ASSERT_EQ(status, MRAA_SUCCESS);

   status = mraa_io_close(desc);
   ASSERT_EQ(status, MRAA_SUCCESS);*/
}

/* Test for a successful IIO init. */
TEST_F(mraa_initio_h_unit, test_iio_init)
{
  /*mraa_io_descriptor* desc;
   m raa_result_t status;

   status = mraa_io_init("ii:0x1", &desc);
   ASSERT_EQ(status, MRAA_SUCCESS);

   status = mraa_io_close(desc);
   ASSERT_EQ(status, MRAA_SUCCESS);*/
}

/* Test for a successful PWM init. */
TEST_F(mraa_initio_h_unit, test_pwm_init)
{
  /*mraa_io_descriptor* desc;
   m raa_result_t status;

   status = mraa_io_init("p:1", &desc);
   ASSERT_EQ(status, MRAA_SUCCESS);

   status = mraa_io_close(desc);
   ASSERT_EQ(status, MRAA_SUCCESS);*/
}

/* Test for a successful SPI init. */
TEST_F(mraa_initio_h_unit, test_spi_init)
{
  /*mraa_io_descriptor* desc;
   m raa_result_t status;

   status = mraa_io_init(" s:0x1:mode2:400000", &desc);
   ASSERT_EQ(status, MRAA_SUCCESS);

   status = mraa_io_close(desc);
   ASSERT_EQ(status, MRAA_SUCCESS);*/
}

/* Test for a successful UART init. */
TEST_F(mraa_initio_h_unit, test_uart_init)
{
  /*mraa_io_descriptor* desc;
   m raa_result_t status;

   status = mraa_io_init("u:0x1:9600:8N1", &desc);
   ASSERT_EQ(status, MRAA_SUCCESS);

   status = mraa_io_close(desc);
   ASSERT_EQ(status, MRAA_SUCCESS);*/
}

/* Test for a successful UART_OW init. */
TEST_F(mraa_initio_h_unit, test_uart_ow_init)
{
  /*mraa_io_descriptor* desc;
   m raa_result_t status;

   status = mraa_io_init("ow:0x1", &desc);
   ASSERT_EQ(status, MRAA_SUCCESS);

   status = mraa_io_close(desc);
   ASSERT_EQ(status, MRAA_SUCCESS);*/
}

/* Test for multiple io initialization and access the structs for
   read/write operations. */
TEST_F(mraa_initio_h_unit, test_multiple_init)
{
   /*mraa_io_descriptor* desc;
   mraa_result_t status;

   status = mraa_io_init("g:13:out:1,g:11:out:1", &desc);
   ASSERT_EQ(status, MRAA_SUCCESS);

   for (int i = 0; i < 2; ++i) {
     ASSERT_EQ(mraa_gpio_read(desc->gpios[i]), 1);
   }

   status = mraa_io_close(desc);
   ASSERT_EQ(status, MRAA_SUCCESS);*/
}

TEST_F(mraa_initio_h_unit, test_leftover_string)
{
   /*mraa_io_descriptor* desc;
   mraa_result_t status;

   status = mraa_io_init("a:1:10,any_string", &desc);
   ASSERT_EQ(status, MRAA_SUCCESS);

   ASSERT_STREQ(desc->leftover_str, "any_string");

   status = mraa_io_close(desc);
   ASSERT_EQ(status, MRAA_SUCCESS);*/
}
