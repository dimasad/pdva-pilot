
/** @file
 * Unit tests of the parameter configuration infrastructure.
 */

#include <fstream>
#include <limits>
#include <unistd.h>

#include <gtest/gtest.h>

#include "../param.h"

using namespace std;
using namespace testing;


typedef enum MAV_PARAM_TYPE mav_param_type_t;

template <typename T> enum MAV_PARAM_TYPE
mav_param_type();

template <> enum MAV_PARAM_TYPE
mav_param_type<int8_t>() {return MAV_PARAM_TYPE_INT8;}

template <> enum MAV_PARAM_TYPE
mav_param_type<int16_t>() {return MAV_PARAM_TYPE_INT16;}

template <> enum MAV_PARAM_TYPE
mav_param_type<int32_t>() {return MAV_PARAM_TYPE_INT32;}

template <> enum MAV_PARAM_TYPE
mav_param_type<uint8_t>() {return MAV_PARAM_TYPE_UINT8;}

template <> enum MAV_PARAM_TYPE
mav_param_type<uint16_t>() {return MAV_PARAM_TYPE_UINT16;}

template <> enum MAV_PARAM_TYPE
mav_param_type<uint32_t>() {return MAV_PARAM_TYPE_UINT32;}

template <> enum MAV_PARAM_TYPE
mav_param_type<float>() {return MAV_PARAM_TYPE_REAL32;}


/// Get value of param_value_union_t
template <typename T> T
punion_value(param_value_union_t);

/// Get int8_t value of param_value_union_t
template <> int8_t
punion_value(param_value_union_t punion) {
  return punion.param_int8;
}

/// Get int16_t value of param_value_union_t
template <> int16_t
punion_value(param_value_union_t punion) {
  return punion.param_int16;
}

/// Get int32_t value of param_value_union_t
template <> int32_t
punion_value(param_value_union_t punion) {
  return punion.param_int32;
}

/// Get uint8_t value of param_value_union_t
template <> uint8_t
punion_value(param_value_union_t punion) {
  return punion.param_uint8;
}

/// Get uint16_t value of param_value_union_t
template <> uint16_t
punion_value(param_value_union_t punion) {
  return punion.param_uint16;
}

/// Get uint32_t value of param_value_union_t
template <> uint32_t
punion_value(param_value_union_t punion) {
  return punion.param_uint32;
}

/// Get float value of param_value_union_t
template <> float
punion_value(param_value_union_t punion) {
  return punion.param_float;
}

/// Set value of param_value_union_t
template <typename T> param_value_union_t 
make_punion(T);

/// Set int8_t value of param_value_union_t
template <> param_value_union_t
make_punion(int8_t value) {
  param_value_union_t punion;
  punion.param_int8 = value;
  return punion;
}

/// Set int16_t value of param_value_union_t
template <> param_value_union_t
make_punion(int16_t value) {
  param_value_union_t punion;
  punion.param_int16 = value;
  return punion;
}

/// Set int32_t value of param_value_union_t
template <> param_value_union_t
make_punion(int32_t value) {
  param_value_union_t punion;
  punion.param_int32 = value;
  return punion;
}

/// Set uint8_t value of param_value_union_t
template <> param_value_union_t
make_punion(uint8_t value) {
  param_value_union_t punion;
  punion.param_uint8 = value;
  return punion;
}

/// Set uint16_t value of param_value_union_t
template <> param_value_union_t
make_punion(uint16_t value) {
  param_value_union_t punion;
  punion.param_uint16 = value;
  return punion;
}

/// Set uint32_t value of param_value_union_t
template <> param_value_union_t
make_punion(uint32_t value) {
  param_value_union_t punion;
  punion.param_uint32 = value;
  return punion;
}

/// Set float value of param_value_union_t
template <> param_value_union_t
make_punion(float value) {
  param_value_union_t punion;
  punion.param_float = value;
  return punion;
}

/// Parameter test fixture
template <typename T> class ParamTest : public Test {};
typedef Types<int8_t,int16_t,int32_t,uint8_t,uint16_t,uint32_t,float>ParamTypes;
TYPED_TEST_CASE(ParamTest, ParamTypes);


/// Test param get/set functions.
TYPED_TEST(ParamTest, GetSet) {
  TypeParam min = numeric_limits<TypeParam>::min();
  TypeParam max = numeric_limits<TypeParam>::max();
  
  TypeParam val = min;
  param_t param = {};
  param.type = mav_param_type<TypeParam>();
  param.data = &val;
  EXPECT_EQ(punion_value<TypeParam>(param_get(&param)), min);
  
  param_value_union_t new_value = make_punion(max);
  EXPECT_EQ(param_set(&param, new_value), STATUS_SUCCESS);
  EXPECT_EQ(max, val);
}

/// Custom parameter type
typedef struct {void *unused; int32_t i;} custom_param_t;

/// Custom parameter getter.
static param_value_union_t
custom_param_getter(enum MAV_PARAM_TYPE type, const char *id, void *data) {
  return (param_value_union_t){.param_int32 = ((custom_param_t*) data)->i};
}

/// Custom parameter setter.
static ret_status_t
custom_param_setter(enum MAV_PARAM_TYPE type, const char *id,
		     void *data, param_value_union_t new_value) {
  ((custom_param_t*) data)->i = new_value.param_int32;
  return STATUS_SUCCESS;
}

/// Test custom parameter getter/setter.
TEST(ParamTest, CustomGetSet) {
  custom_param_t val = {.unused = NULL, .i=3};
  param_t param = {
    .type = MAV_PARAM_TYPE_INT32,
    .id = {0},
    .data = &val,
    .getter = &custom_param_getter,
    .setter = &custom_param_setter
  };
  EXPECT_EQ(param_get(&param).param_int32, val.i);

  param_value_union_t new_value = {.param_int32 = -1000};
  EXPECT_EQ(param_set(&param, new_value), STATUS_SUCCESS);
  EXPECT_EQ(val.i, new_value.param_int32);
}

/// Parameter handler test fixture
template <typename T> class ParamHandlerTest : public Test {};
TYPED_TEST_CASE(ParamHandlerTest, ParamTypes);


/// Test param handler load/save functions.
TYPED_TEST(ParamHandlerTest, LoadSave) {
  TypeParam min = numeric_limits<TypeParam>::min();
  TypeParam max = numeric_limits<TypeParam>::max();
  TypeParam save_val = min;
  
  enum MAV_PARAM_TYPE type = mav_param_type<TypeParam>();
  param_handler_t save_handler;
  param_handler_init(&save_handler, 1);
  param_register(&save_handler, type, "test_param", &save_val, NULL, NULL);

  const char *test_file = "param_handler_test.dat";
  unlink(test_file);
  EXPECT_EQ(param_save(&save_handler, test_file), STATUS_SUCCESS);
  param_handler_destroy(&save_handler);
  
  TypeParam load_val = max;
  param_handler_t load_handler;
  param_handler_init(&load_handler, 0);
  param_register(&load_handler, type, "test_param", &load_val, NULL, NULL);
  
  EXPECT_EQ(param_load(&load_handler, test_file), STATUS_SUCCESS);
  EXPECT_EQ(load_val, save_val);
  param_handler_destroy(&load_handler);
}

/// Test loading of pdva-pilot configuration object.
TEST(PDVAConfigTest, Load) {
  const char *config_file_name = "pdva_config_test.cfg";
  const unsigned sysid = 255;
  
  ofstream config_file(config_file_name, ios::trunc);
  config_file << "sysid = " << sysid << ";" << endl;
  config_file.close();
  
  pdva_pilot_config_t config = {};
  pdva_config_init(&config);
  EXPECT_EQ(pdva_config_load(&config, config_file_name), STATUS_SUCCESS);
  EXPECT_EQ(config.sysid, sysid);
  
  pdva_config_destroy(&config);
}
