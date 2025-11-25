# ENSC351_Project
ENSC351-Real Time Embedded Systems

HAL (GPIO/ADC/SPI)
      ↓
Position/Speed Estimator  <─── chooses HALL or BEMF mode
      ↓
Commutation + Current/Speed PI
      ↓
PWM outputs (via HAL)
