/***************************************************************************//**
* @file    main.c
* @version 1.0.0
*
* @brief Azure Sphere Avnet Starter Kit LPS22HH sensor usage example.
*
* @author Jaroslav Groman
*
*******************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <string.h>

#include "applibs_versions.h"   // API struct versions to use for applibs APIs
#include <applibs/log.h>
#include <applibs/gpio.h>
#include <applibs/i2c.h>

// Import project hardware abstraction from project property 
// "Target Hardware Definition Directory"
#include <hw/project_hardware.h>

// Using a single-thread event loop pattern based on Epoll and timerfd
#include "epoll_timerfd_utilities.h"

#include "lib_lps22hh.h"

/*******************************************************************************
* Forward declarations of private functions
*******************************************************************************/

/**
 * @brief Application termination handler.
 *
 * Signal handler for termination requests. This handler must be
 * async-signal-safe.
 *
 * @param signal_number
 *
 */
static void
termination_handler(int signal_number);

/**
 * @brief Initialize signal handlers.
 *
 * Set up SIGTERM termination handler.
 *
 * @return 0 on success, -1 otherwise.
 */
static int
init_handlers(void);

/**
 * @brief Initialize peripherals.
 *
 * Initialize all peripherals used by this project.
 *
 * @return 0 on success, -1 otherwise.
 */
static int
init_peripherals(I2C_InterfaceId isu_id);

/**
 *
 */
static void
close_peripherals_and_handlers(void);

/**
 * @brief Button1 press handler
 */
static void
button1_press_handler(void);

/**
 * @brief Timer event handler for polling button states
 */
static void
button_timer_event_handler(EventData *event_data);

/*******************************************************************************
* Global variables
*******************************************************************************/

// Termination state flag
static volatile sig_atomic_t gb_is_termination_requested = false;

static int g_fd_i2c = -1;
static int g_fd_epoll = -1;
static int g_fd_poll_timer_button = -1;
static int g_fd_gpio_button1 = -1;

static GPIO_Value_Type g_state_button1 = GPIO_Value_High;

static EventData g_event_data_button = {          // Event handler data
    .eventHandler = &button_timer_event_handler // Populate only this field
};

static stmdev_ctx_t *gp_lps22hh_ctx;

/*******************************************************************************
* Function definitions
*******************************************************************************/

int
main(int argc, char *argv[])
{
    const struct timespec SLEEP_TIME_5S = { 5, 0 };

    lps22hh_reg_t lps22hh_reg;
    float lps_temp_degc;
    float lps_pressure_hpa;

    uint8_t data_raw_temperature[2];

    // There are only 3 bytes of pressure data but we will cast buffer contents
    // to int32_t later
    uint8_t data_raw_pressure[4];

    Log_Debug("\n*** Starting ***\n");
    Log_Debug("Press Button 1 to exit.\n");

    gb_is_termination_requested = false;

    // Initialize handlers
    if (init_handlers() != 0)
    {
        gb_is_termination_requested = true;
    }

    // Initialize peripherals
    if (!gb_is_termination_requested)
    {
        if (init_peripherals(PROJECT_ISU2_I2C) != 0)
        {
            gb_is_termination_requested = true;
        }
    }

    // Main program
    if (!gb_is_termination_requested)
    {
        Log_Debug("Waiting for timer events\n");

        // Main program loop
        while (!gb_is_termination_requested)
        {
            // Read LPS22HH status
            lps22hh_read_reg(gp_lps22hh_ctx, LPS22HH_STATUS, (uint8_t *)&lps22hh_reg, 1);

            // Check that new pressure and temperature data are ready
            if ((lps22hh_reg.status.p_da == 1) && (lps22hh_reg.status.t_da == 1))
            {
                data_raw_pressure[3] = 0;
                lps22hh_pressure_raw_get(gp_lps22hh_ctx, data_raw_pressure);
                lps_pressure_hpa = lps22hh_from_lsb_to_hpa(*(int32_t *)data_raw_pressure);
                Log_Debug("LPS22HH: Pressure     [hPa] : %.2f\n", lps_pressure_hpa);

                lps22hh_temperature_raw_get(gp_lps22hh_ctx, data_raw_temperature);
                lps_temp_degc = lps22hh_from_lsb_to_celsius(*(int16_t *)data_raw_temperature);
                Log_Debug("LPS22HH: Temperature  [degC]: %.2f\n\n", lps_temp_degc);
            }

            // Handle timers
            if (WaitForEventAndCallHandler(g_fd_epoll) != 0)
            {
                gb_is_termination_requested = true;
            }

            nanosleep(&SLEEP_TIME_5S, NULL);
        }

        Log_Debug("Leaving main loop\n");
    }

    close_peripherals_and_handlers();

    Log_Debug("*** Terminated ***\n");
    return 0;
}

/*******************************************************************************
* Private function definitions
*******************************************************************************/

static void
termination_handler(int signal_number)
{
    gb_is_termination_requested = true;
}

static int
init_handlers(void)
{
    Log_Debug("Init Handlers\n");

    int result = -1;

    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = termination_handler;
    result = sigaction(SIGTERM, &action, NULL);
    if (result != 0) 
    {
        // Failed to create signal action handler
        Log_Debug("ERROR: %s - sigaction: errno=%d (%s)\n",
            __FUNCTION__, errno, strerror(errno));
    }

    if (result == 0)
    {
        g_fd_epoll = CreateEpollFd();
        if (g_fd_epoll < 0)
        {
            // Failed to create epoll
            result = -1;
        }
    }

    return result;
}

static int
init_peripherals(I2C_InterfaceId isu_id)
{
    int result = -1;

    // Initialize I2C bus
    Log_Debug("Init I2C\n");
    g_fd_i2c = I2CMaster_Open(isu_id);
    if (g_fd_i2c < 0) 
    {
        Log_Debug("ERROR: I2CMaster_Open: errno=%d (%s)\n",
            errno, strerror(errno));
    }
    else
    {
        result = I2CMaster_SetBusSpeed(g_fd_i2c, I2C_BUS_SPEED_STANDARD);
        if (result != 0) 
        {
            Log_Debug("ERROR: I2CMaster_SetBusSpeed: errno=%d (%s)\n",
                errno, strerror(errno));
        }
        else
        {
            result = I2CMaster_SetTimeout(g_fd_i2c, 100);
            if (result != 0) 
            {
                Log_Debug("ERROR: I2CMaster_SetTimeout: errno=%d (%s)\n",
                    errno, strerror(errno));
            }
        }
    }

    // Initialize LPS22HH sensor
    if (result != -1)
    {
        Log_Debug("Init LPS22HH\n");
        gp_lps22hh_ctx = lps22hh_open_via_lsm6dso(g_fd_i2c);
        if (!gp_lps22hh_ctx)
        {
            Log_Debug("Error initializing LPS22HH sensor\n");
        }
    }

    // Initialize development kit button GPIO
    // Open button 1 GPIO as input
    if (result != -1)
    {
        Log_Debug("Opening PROJECT_BUTTON_1 as input.\n");
        g_fd_gpio_button1 = GPIO_OpenAsInput(PROJECT_BUTTON_1);
        if (g_fd_gpio_button1 < 0) 
        {
            Log_Debug("ERROR: Could not open button GPIO: %s (%d).\n",
                strerror(errno), errno);
            result = -1;
        }
    }

    // Create timer for button press check
    if (result != -1)
    {
        struct timespec button_press_check_period = { 0, 1000000 };
        g_fd_poll_timer_button = CreateTimerFdAndAddToEpoll(g_fd_epoll,
            &button_press_check_period, &g_event_data_button, EPOLLIN);
        if (g_fd_poll_timer_button < 0)
        {
            Log_Debug("ERROR: Could not create button poll timer: %s (%d).\n",
                strerror(errno), errno);
            result = -1;
        }
    }

    return result;
}

static void
close_peripherals_and_handlers(void)
{
    // Close LPS22HH sensor
    Log_Debug("Close LPS22HH\n");
    if (gp_lps22hh_ctx)
    {
        lps22hh_close(gp_lps22hh_ctx);
    }

    // Close I2C
    if (g_fd_i2c)
    {
        close(g_fd_i2c);
    }
}

static void
button1_press_handler(void)
{
    Log_Debug("Button1 pressed.\n");
    gb_is_termination_requested = true;
}

static void
button_timer_event_handler(EventData *event_data)
{
    GPIO_Value_Type new_btn1_state;
    bool b_is_all_ok = true;

    // Consume timer event
    if (ConsumeTimerFdEvent(g_fd_poll_timer_button) != 0)
    {
        // Failed to consume timer event
        gb_is_termination_requested = true;
        b_is_all_ok = false;
    }

    // Check for a button1 press
    if (b_is_all_ok && (GPIO_GetValue(g_fd_gpio_button1, &new_btn1_state) != 0))
    {
        // Failed to get GPIO pin value
        Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n",
            strerror(errno), errno);
        gb_is_termination_requested = true;
        b_is_all_ok = false;
    }

    if (b_is_all_ok && (new_btn1_state != g_state_button1))
    {
        if (new_btn1_state == GPIO_Value_Low)
        {
            button1_press_handler();
        }
        g_state_button1 = new_btn1_state;
    }

    return;
}

/* [] END OF FILE */
