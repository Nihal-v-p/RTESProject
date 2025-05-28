# Wearable Parkinsonian Tremor Detector on STM32

This project detects Parkinsonian resting tremors (3–6 Hz) using the built-in L3GD20 gyroscope on the STM32F429 Discovery board. It is implemented in C++ with PlatformIO and utilizes CMSIS-DSP for real-time FFT analysis. Tremor intensity is visualized on the onboard LCD using color codes.

We also applied a **moving average filter** to smooth angular velocity data before performing frequency domain analysis, improving noise resilience in real-time detection.

### 📽️ Demo Video
[![Watch the demo](https://img.youtube.com/vi/AfLE_Fn-GYE/0.jpg)]([https://www.youtube.com/watch?v=YOUTUBE_VIDEO_ID](https://www.youtube.com/watch?v=AfLE_Fn-GYE))

> Click the thumbnail to view the demo on YouTube.

### 🚀 Tech Stack
- **Platform:** STM32F429 Discovery Board
- **Language:** C++
- **Framework:** PlatformIO
- **Libraries:** CMSIS-DSP
- **Signal Processing:** FFT, Moving Average Filter
- **Communication:** SPI (for gyroscope data)
