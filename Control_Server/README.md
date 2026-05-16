## SIMBA Mission Control Server

### Project Overview:
This sub-system of the SIMBA project acts as the server that processes the web client requests. Based on the request, the server will
use the different component libraries to perform various actions like moving the rover in differrent directions or starting the
arm pick up sequence.

### Built with:
* [![C](https://img.shields.io/badge/C-00599C?style=for-the-badge&logo=c&logoColor=white)](https://en.wikipedia.org/wiki/C_(programming_language))


## Getting Started
### SSH into Rover and Load Bitstream

1. Open a terminal and enter the following command
   ```sh
   ssh ubuntu@ipAddress
   ```
2. Enter the password according to team credentials

3. Navigate into the bitream folder
    ```sh
   cd /home/ubuntu/SIMBA_C_Driver/bitstream
   ```
3. Load the bitstream
    ```sh
   python3 load.py
   ```
   
### Start Rover Server

1. Navigate back to the Control-Server 
   ```sh
   cd ../../Control-Server
   ```
2. Compile the code
   ```
   make all
   ```
4. Run the server. The port is hardcoded to 6500
   ```
   ./server
   ```


