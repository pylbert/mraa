#include "gtest/gtest.h"
#include "mraa/initio.h"

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

/* Test for a successful mraa_init */
TEST_F(mraa_initio_h_unit, test_aio_init)
{
  mraa_io_descriptor* desc;
  mraa_result_t status;

  status = mraa_io_init("a:0:10", &desc);
  ASSERT_EQ(status, MRAA_SUCCESS);

  status = mraa_io_close(desc);
  ASSERT_EQ(status, MRAA_SUCCESS);
}

/* Test for a basic mraa init/deinit */
TEST_F(mraa_initio_h_unit, test_2)
{
  ASSERT_EQ(MRAA_SUCCESS,  mraa_init());
  mraa_deinit();
}

/* Test the C exposed common methods */
TEST_F(mraa_initio_h_unit, test_3)
{

}
