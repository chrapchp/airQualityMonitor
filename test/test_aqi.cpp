#include <Arduino.h>
#include <unity.h>
#include <aqi.h>

AQI aqi = AQI();
// void setUp(void) {
// // set stuff up here
// }

// void tearDown(void) {
// // clean stuff up here
// }

void test_function_PM25_index_good(void)
{
    AQI_Data result = aqi.get_AQI_PM25(5);

    TEST_ASSERT_EQUAL_FLOAT(20.8333, result.index);
}

void test_function_PM25_category_good(void)
{
    AQI_Data result = aqi.get_AQI_PM25(5);
    TEST_ASSERT_EQUAL(GOOD, result.category);
}

void test_function_PM25_index_moderate(void)
{
    AQI_Data result = aqi.get_AQI_PM25(15);

    TEST_ASSERT_EQUAL_FLOAT(57.09871, result.index);
}

void test_function_PM25_category_moderate(void)
{
    AQI_Data result = aqi.get_AQI_PM25(15);
    TEST_ASSERT_EQUAL(MODERATE, result.category);
}

void test_function_PM25_index_unhealthySG(void)
{
    AQI_Data result = aqi.get_AQI_PM25(36.3);
    TEST_ASSERT_EQUAL_FLOAT(102.9698, result.index);
}

void test_function_PM25_category_unhealthySG(void)
{
    AQI_Data result = aqi.get_AQI_PM25(35.5);
    TEST_ASSERT_EQUAL(UNHEALTHY_SENSITVE_GROUPS, result.category);
}

void test_function_PM25_index_unhealthy(void)
{
    AQI_Data result = aqi.get_AQI_PM25(131.1);
    TEST_ASSERT_EQUAL_FLOAT(190.0348, result.index);
}

void test_function_PM25_category_unhealthy(void)
{
    AQI_Data result = aqi.get_AQI_PM25(55.5);
    TEST_ASSERT_EQUAL(UNHEALTHY, result.category);
}

void test_function_PM25_index_very_unhealthy(void)
{
    AQI_Data result = aqi.get_AQI_PM25(165.5);
    TEST_ASSERT_EQUAL_FLOAT(215.865, result.index);
}

void test_function_PM25_category_very_unhealthy(void)
{
    AQI_Data result = aqi.get_AQI_PM25(250.4);
    TEST_ASSERT_EQUAL(VERY_HUNHEALTHY, result.category);
}

void test_function_PM25_index_hazardous1(void)
{
    AQI_Data result = aqi.get_AQI_PM25(350.4);
    TEST_ASSERT_EQUAL_FLOAT(400.000, result.index);
}

void test_function_PM25_category_hazardous1(void)
{
    AQI_Data result = aqi.get_AQI_PM25(266.0);
    TEST_ASSERT_EQUAL(HAZARDOUS, result.category);
}

void test_function_PM25_index_hazardous2(void)
{
    AQI_Data result = aqi.get_AQI_PM25(350.5);
    TEST_ASSERT_EQUAL_FLOAT(401.000, result.index);
}

void test_function_PM25_category_hazardous2(void)
{
    AQI_Data result = aqi.get_AQI_PM25(366.0);
    TEST_ASSERT_EQUAL(HAZARDOUS, result.category);
}

void test_function_PM25_error_low(void)
{
    AQI_Data result = aqi.get_AQI_PM25(-1.0);
    TEST_ASSERT_EQUAL(UNKNOWN, result.category);
}

void test_function_PM25_error_high(void)
{
    AQI_Data result = aqi.get_AQI_PM25(500.5);
    TEST_ASSERT_EQUAL(UNKNOWN, result.category);
}

void testPM25()
{
    RUN_TEST(test_function_PM25_index_good);
    RUN_TEST(test_function_PM25_category_good);

    RUN_TEST(test_function_PM25_index_moderate);
    RUN_TEST(test_function_PM25_category_moderate);

    RUN_TEST(test_function_PM25_index_unhealthySG);
    RUN_TEST(test_function_PM25_category_unhealthySG);

    RUN_TEST(test_function_PM25_index_unhealthy);
    RUN_TEST(test_function_PM25_category_unhealthy);

    RUN_TEST(test_function_PM25_index_very_unhealthy);
    RUN_TEST(test_function_PM25_category_very_unhealthy);

    RUN_TEST(test_function_PM25_index_hazardous1);
    RUN_TEST(test_function_PM25_category_hazardous1);

    RUN_TEST(test_function_PM25_index_hazardous2);
    RUN_TEST(test_function_PM25_category_hazardous2);

    RUN_TEST(test_function_PM25_error_low);
    RUN_TEST(test_function_PM25_error_high);
}

void test_function_PM10_index_good(void)
{
    AQI_Data result = aqi.get_AQI_PM10(54.0);

    TEST_ASSERT_EQUAL_FLOAT(50.000, result.index);
}

void test_function_PM10_category_good(void)
{
    AQI_Data result = aqi.get_AQI_PM10(15);
    TEST_ASSERT_EQUAL(GOOD, result.category);
}

void test_function_PM10_index_moderate(void)
{
    AQI_Data result = aqi.get_AQI_PM10(56.7);

    TEST_ASSERT_EQUAL_FLOAT(51.8414, result.index);
}

void test_function_PM10_category_moderate(void)
{
    AQI_Data result = aqi.get_AQI_PM10(154);
    TEST_ASSERT_EQUAL(MODERATE, result.category);
}

void test_function_PM10_index_unhealthySG(void)
{
    AQI_Data result = aqi.get_AQI_PM10(220.4);
    TEST_ASSERT_EQUAL_FLOAT(133.3697, result.index);
}

void test_function_PM10_category_unhealthySG(void)
{
    AQI_Data result = aqi.get_AQI_PM10(155);
    TEST_ASSERT_EQUAL(UNHEALTHY_SENSITVE_GROUPS, result.category);
}


void test_function_PM10_index_unhealthy(void)
{
    AQI_Data result = aqi.get_AQI_PM10(266.1);
    TEST_ASSERT_EQUAL_FLOAT(156.4939, result.index);
}

void test_function_PM10_category_unhealthy(void)
{
    AQI_Data result = aqi.get_AQI_PM10(354.0);
    TEST_ASSERT_EQUAL(UNHEALTHY, result.category);
}

void test_function_PM10_index_very_unhealthy(void)
{
    AQI_Data result = aqi.get_AQI_PM10(378.3);
    TEST_ASSERT_EQUAL_FLOAT(234.4304, result.index);
}

void test_function_PM10_category_very_unhealthy(void)
{
    AQI_Data result = aqi.get_AQI_PM10(355.0);
    TEST_ASSERT_EQUAL(VERY_HUNHEALTHY, result.category);
}


void test_function_PM10_index_hazardous1(void)
{
    AQI_Data result = aqi.get_AQI_PM10(503.5);
    TEST_ASSERT_EQUAL_FLOAT(399.3734, result.index);
}

void test_function_PM10_category_hazardous1(void)
{
    AQI_Data result = aqi.get_AQI_PM10(425.0);
    TEST_ASSERT_EQUAL(HAZARDOUS, result.category);
}

void test_function_PM10_index_hazardous2(void)
{
    AQI_Data result = aqi.get_AQI_PM10(505.1);
    TEST_ASSERT_EQUAL_FLOAT(401.100, result.index);
}

void test_function_PM10_category_hazardous2(void)
{
    AQI_Data result = aqi.get_AQI_PM10(604.0);
    TEST_ASSERT_EQUAL(HAZARDOUS, result.category);
}

void test_function_PM10_error_low(void)
{
    AQI_Data result = aqi.get_AQI_PM10(-1.0);
    TEST_ASSERT_EQUAL(UNKNOWN, result.category);
}

void test_function_PM10_error_high(void)
{
    AQI_Data result = aqi.get_AQI_PM10(604.1);
    TEST_ASSERT_EQUAL(UNKNOWN, result.category);
}


void testPM10()
{
    RUN_TEST(test_function_PM10_index_good);
    RUN_TEST(test_function_PM10_category_good);

    RUN_TEST(test_function_PM10_index_moderate);
    RUN_TEST(test_function_PM10_category_moderate);

    RUN_TEST(test_function_PM10_index_unhealthySG);
    RUN_TEST(test_function_PM10_category_unhealthySG);

    RUN_TEST(test_function_PM10_index_unhealthy);
    RUN_TEST(test_function_PM10_category_unhealthy);

    RUN_TEST(test_function_PM10_index_very_unhealthy);
    RUN_TEST(test_function_PM10_category_very_unhealthy);

    RUN_TEST(test_function_PM10_index_hazardous1);
    RUN_TEST(test_function_PM10_category_hazardous1);

    RUN_TEST(test_function_PM10_index_hazardous2);
    RUN_TEST(test_function_PM10_category_hazardous2);

    RUN_TEST(test_function_PM10_error_low);
    RUN_TEST(test_function_PM10_error_high);
}

void setup()
{
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);

    UNITY_BEGIN();
    testPM10();
    testPM25();
   

    UNITY_END();
}

void loop()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
}