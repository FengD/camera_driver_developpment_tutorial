# Camera Driver Developpment Tutorial

As a software engineer, I have completed the development of a camera driver as a beginner on both the Raspberry Pi and Jetson platforms. I want to write a detailed tutorial to guide beginners on how to develop a camera driver on SoC platforms, helping them achieve camera usage freedom. This tutorial is particularly suitable for beginners, even for those with no prior knowledge of driver development or further camera development. The article will provide an understanding of the step-by-step process of developing a camera driver from scratch, as well as important considerations during camera usage.

For those interested in using cameras for tasks like robotics or deep learning-based detection, this tutorial will also offer valuable insights. It will help you understand how to set up the camera for real-time image capture, which can be applied to projects such as object detection, autonomous navigation, or other AI-driven tasks. Additionally, I will include examples and source code for getting the IMX390C and IMX490C working on Jetson and Raspberry Pi.

# Content
* 0. How a camera work
* 1. The Steps of the Camera Driver Developpment
* 2. The Best practice on Jetson Orin nano and Raspberry Pi 4B
* 3. Demo & Annexes

# 0. How a camera work
* With the picture below you could find exactly the pipeline of a camera. If we want to get the images from the SoC. The hardware pass throught from input to output are `Lens -> Sensor -> ISP(Optional on Camera or On SoC) -> Ser/Des -> SoC`.

* The Sensor, ISP, Ser/Des could control or config by I2C and get the forward data by MIPI, which is the logic of the camera driver.

![camera](./sources/camera.png)

# 1. The Steps of the Camera Driver Developpment

1. Config & Compile the Device Tree. i2c command could be used to check whether the device(sensor, ser/des) appears on the I2c Bus.


# 2. The Best practice on Jetson Orin nano and Raspberry Pi 4B

# 3. Demo & Annexes
