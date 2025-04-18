# HyFlow: Hydration Monitoring Device

HyFlow is a real-time hydration monitoring system designed to help users track and maintain optimal hydration levels. The project combines hardware sensors, BLE communication, and a modern web dashboard to provide continuous feedback and actionable insights about your hydration status.

## Project Overview

- **Hardware**: Custom wearable device using sensors (GSR, IMU, MAX30102) to collect physiological data related to hydration.
- **Firmware**: Arduino-based code for sensor calibration, data acquisition, and BLE transmission.
- **Backend**: Node.js/Express server for user authentication and MongoDB for storing user data.
- **Frontend**: React-based dashboard for data visualization, user login/registration, and real-time hydration status alerts.
- **Data Analysis**: Hydration status is predicted and visualized using historical sensor data.

## Features

- Real-time hydration status monitoring (Hydrated, Mildly Dehydrated, Dehydrated)
- User registration and login system
- Interactive dashboard with graphs and status notifications
- Data filtering by time (24h, 7d, 30d)
- BLE-based communication between device and web app

## Team

This project was created as part of our year-long Design and Fabrication course project by the following team members:

- **Gaurav Kokila** (that's me)
- Shushim Gautamkumar
- Anushka
- Harshita
- Nikita Nishiprava
- Himanshi Arora

## Getting Started

1. **Hardware**: Upload the Arduino firmware to your device.
2. **Backend**: Start the Node.js server (`npm install` and `npm start` in the backend folder).
3. **Frontend**: Start the React app (`npm install` and `npm run dev` in the frontend folder).
4. **BLE Connection**: Pair your device with the web app via BLE for live data streaming.

## Folder Structure

- `completecode/` - Arduino firmware for the wearable device
- `EDP/backend/` - Node.js/Express backend code
- `EDP/frontend/` - React frontend code
- `hydration_dataset.csv` - Collected hydration data

## Acknowledgements

Special thanks to our mentors and everyone who supported us throughout this project.

---

Stay hydrated, stay healthy!
