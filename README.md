# MakeOhio 2026 Smart Auto-Sorting Trash Bin

We built an **AI-powered smart trash bin** that helps users correctly sort waste and recycling. Using a camera and computer vision, the system identifies the object and automatically routes it to the correct bin.

The goal is to **reduce recycling contamination** and make waste sorting easier and more accurate.

---

## How It Works

1. A user places an item in the bin.
2. IR sensors detect which side the item was placed on.
3. The ESP32 camera captures an image of the item.
4. The image is sent to Google Vision API for classification.
5. The system determines if the item is **recyclable or waste**.
6. A servo rotates to direct the item into the correct bin.
7. LEDs and sound provide feedback to the user.

---

## Features

- AI object recognition using Google Vision API  
- Automatic waste vs recycling sorting  
- Camera-based item detection  
- Servo-driven sorting mechanism  
- LED and audio feedback system  
- Real-time classification and response  

---

## Hardware

- ESP32 WROOM-32E  
- ESP32 Camera Extension (OV2640)  
- IR obstacle sensors (2)  
- Servo motor  
- WS2812 LED strip  
- Buzzer  
- Power supply  

---

## Software

- Arduino framework  
- ESP32 Camera library  
- Google Vision API  
- ArduinoJson  
- Adafruit NeoPixel  

---

## Future Improvements

- Faster on-device classification (edge AI)
- Weight sensor for material detection
- Data aggregation and waste analytics dashboard
- Improved object detection models

---

## Authors

Developed as part of a hackathon project focused on **smart waste management and sustainability**.
