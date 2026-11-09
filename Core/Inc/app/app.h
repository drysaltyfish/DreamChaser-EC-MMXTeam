#ifndef APP_H
#define APP_H

/**
 * Builds the application objects and starts the protocol tasks.
 *
 * Called once from the CubeMX default task after the scheduler is running.
 * The caller must delete itself afterwards.
 */
void App_Init(void);

#endif /* APP_H */
