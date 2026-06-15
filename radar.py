import serial
import time
import numpy as np
from sklearn.cluster import OPTICS


# Change the configuration file name
configFileName = 'config.cfg'
CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2 ** 15, dtype='uint8')
byteBufferLength = 0

# ------------------------------------------------------------------

# Function to configure the serial ports and send the data from
# the configuration file to the radar
def serialConfig(configFileName):
    global CLIport
    global Dataport
    # Open the serial ports for the configuration and the data ports

    # macbook
    CLIport = serial.Serial('/dev/tty.usbserial-010821FD0', 115200)
    Dataport = serial.Serial('/dev/tty.usbserial-010821FD1', 921600)
    #JETSON
    #CLIport = serial.Serial('/dev/ttyUSB0', 115200)
    #Dataport = serial.Serial('/dev/ttyUSB1', 921600)

    # Windows
    # CLIport = serial.Serial('COM3', 115200)
    # Dataport = serial.Serial('COM4', 921600)

    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        CLIport.write((i + '\n').encode())
        print(i)
        time.sleep(0.01)

    return CLIport, Dataport


# ------------------------------------------------------------------

# Function to parse the data inside the configuration file
def parseConfigFile(configFileName):
    configParameters = {}  # Initialize an empty dictionary to store the configuration parameters

    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        # Split the line
        splitWords = i.split(" ")
        # Hard code the number of antennas, change if other configuration is used
        numRxAnt = 4
        numTxAnt = 3

        # Get the information about the profile configuration
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            numAdcSamplesRoundTo2 = 1;

            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2;

            digOutSampleRate = int(splitWords[11]);

        # Get the information about the frame configuration
        elif "frameCfg" in splitWords[0]:

            chirpStartIdx = int(splitWords[1]);
            chirpEndIdx = int(splitWords[2]);
            numLoops = int(splitWords[3]);
            numFrames = int(splitWords[4]);
            framePeriodicity = int(splitWords[5]);

    # Combine the read data to obtain the configuration parameters
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (
                2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (
                2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["dopplerResolutionMps"] = 3e8 / (
                2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate) / (2 * freqSlopeConst * 1e3)
    configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)

    return configParameters


# ------------------------------------------------------------------

def readAndParseData68xx(Dataport, configParameters):
    global byteBuffer, byteBufferLength
    global num_moving_point, num_cluster

    # Constants
    MMWDEMO_UART_MSG_DETECTED_POINTS = 1
    maxBufferSize = 2 ** 15
    pointLengthInBytes = 16
    magicWord = np.array([2, 1, 4, 3, 6, 5, 8, 7], dtype='uint8')
    HEADER_SIZE = 40                 # 8 magic + 8*4 header 字段
    MAX_TLV_COUNT = 16               # 合理上限, 实际通常 <= 6
    MAX_DETECTED_OBJ = 500           # 按你雷达配置估, 留足够余量
    word = np.array([1, 2 ** 8, 2 ** 16, 2 ** 24], dtype=np.uint32)

    data_status = 0
    frameNumber = 0
    det_obj_set = {}

    # ---------- 1. 读串口 + 防止缓冲区撑满 ----------
    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype='uint8')
    byteCount = len(byteVec)

    if byteBufferLength + byteCount >= maxBufferSize:
        # 缓冲区要满了：丢掉最旧的数据, 而不是丢新数据
        if byteCount >= maxBufferSize:
            byteBuffer[:] = byteVec[-maxBufferSize:]
            byteBufferLength = maxBufferSize
        else:
            keep = maxBufferSize - byteCount
            byteBuffer[:keep] = byteBuffer[byteBufferLength - keep:byteBufferLength]
            byteBuffer[keep:keep + byteCount] = byteVec
            byteBufferLength = keep + byteCount
    else:
        byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec
        byteBufferLength += byteCount

    if byteBufferLength < HEADER_SIZE:
        return 0, 0, {}, num_moving_point, num_cluster

    # ---------- 2. 找 magic word (遍历所有候选, 不是只看第一个) ----------
    candidates = np.where(byteBuffer[:byteBufferLength - 7] == magicWord[0])[0]
    startIdx = -1
    for loc in candidates:
        if np.array_equal(byteBuffer[loc:loc + 8], magicWord):
            startIdx = int(loc)
            break

    if startIdx < 0:
        # 没有 magic word, 保留末尾 7 字节 (可能是半个 magic word)
        tail = min(7, byteBufferLength)
        byteBuffer[:tail] = byteBuffer[byteBufferLength - tail:byteBufferLength]
        byteBufferLength = tail
        return 0, 0, {}, num_moving_point, num_cluster

    # 丢掉 magic word 之前的数据
    if startIdx > 0:
        byteBuffer[:byteBufferLength - startIdx] = byteBuffer[startIdx:byteBufferLength]
        byteBufferLength -= startIdx

    if byteBufferLength < HEADER_SIZE:
        return 0, 0, {}, num_moving_point, num_cluster

    # ---------- 3. 读 totalPacketLen 并做合理性检查 ----------
    totalPacketLen = int(np.matmul(byteBuffer[12:16].astype(np.uint32), word))

    if totalPacketLen < HEADER_SIZE or totalPacketLen > maxBufferSize:
        # 长度非法 -> 说明这个 magic word 是伪命中, 跳过 1 字节重找
        byteBuffer[:byteBufferLength - 1] = byteBuffer[1:byteBufferLength]
        byteBufferLength -= 1
        return 0, 0, {}, num_moving_point, num_cluster

    # 整包还没到齐, 下一轮再来
    if byteBufferLength < totalPacketLen:
        return 0, 0, {}, num_moving_point, num_cluster

    # ---------- 4. 解析 header ----------
    idX = 8                                                     # 跳 magic
    idX += 4                                                    # version
    idX += 4                                                    # totalPacketLen (已读)
    idX += 4                                                    # platform
    frameNumber = int(np.matmul(byteBuffer[idX:idX + 4].astype(np.uint32), word)); idX += 4
    idX += 4                                                    # timeCpuCycles
    numDetectedObj = int(np.matmul(byteBuffer[idX:idX + 4].astype(np.uint32), word)); idX += 4
    numTLVs        = int(np.matmul(byteBuffer[idX:idX + 4].astype(np.uint32), word)); idX += 4
    idX += 4                                                    # subFrameNumber

    # header 字段合理性 — 有一个不对就丢掉整包重新同步
    if not (0 <= numTLVs <= MAX_TLV_COUNT) or not (0 <= numDetectedObj <= MAX_DETECTED_OBJ):
        byteBuffer[:byteBufferLength - totalPacketLen] = byteBuffer[totalPacketLen:byteBufferLength]
        byteBuffer[byteBufferLength - totalPacketLen:] = 0
        byteBufferLength -= totalPacketLen
        return 0, 0, {}, num_moving_point, num_cluster

    # ---------- 5. 解析 TLV (所有访问前都检查边界) ----------
    parse_ok = True
    for _ in range(numTLVs):
        if idX + 8 > totalPacketLen:
            parse_ok = False
            break
        tlv_type   = int(np.matmul(byteBuffer[idX:idX + 4].astype(np.uint32), word)); idX += 4
        tlv_length = int(np.matmul(byteBuffer[idX:idX + 4].astype(np.uint32), word)); idX += 4

        if tlv_length < 0 or idX + tlv_length > totalPacketLen:
            parse_ok = False
            break

        tlv_start = idX

        if tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS:
            expected = numDetectedObj * pointLengthInBytes
            if expected > tlv_length:
                parse_ok = False
                break

            x = np.zeros(numDetectedObj, dtype=np.float32)
            y = np.zeros(numDetectedObj, dtype=np.float32)
            z = np.zeros(numDetectedObj, dtype=np.float32)
            velocity = np.zeros(numDetectedObj, dtype=np.float32)

            for n in range(numDetectedObj):
                x[n]        = byteBuffer[idX:idX + 4].view(dtype=np.float32); idX += 4
                y[n]        = byteBuffer[idX:idX + 4].view(dtype=np.float32); idX += 4
                z[n]        = byteBuffer[idX:idX + 4].view(dtype=np.float32); idX += 4
                velocity[n] = byteBuffer[idX:idX + 4].view(dtype=np.float32); idX += 4

            det_obj_set = {"numObj": numDetectedObj, "x": x, "y": y, "z": z, "velocity": velocity}

            # 你原来的聚类逻辑, 加了 try/except 和空数组保护
            num_moving_point, num_cluster = 0, 0
            coordinate = np.column_stack((x, y, z))
            coordinate = coordinate[~np.isnan(coordinate).any(axis=1)]
            moving = [i for i in range(len(velocity)) if abs(float(velocity[i])) > 0]
            if moving:
                num_moving_point = len(moving)
            if numDetectedObj >= 10 and len(coordinate) >= 2:
                try:
                    clustering = OPTICS(min_samples=2, max_eps=1, xi=0.05).fit(coordinate)
                    labels = np.unique(clustering.labels_).astype(int)
                    if np.sum(labels == -1) > 0 and len(labels) > 1:
                        num_cluster = len(labels) - 1
                    elif np.sum(labels == -1) > 0:
                        num_cluster = 0
                    else:
                        num_cluster = len(labels)
                except Exception as e:
                    print(f'[OPTICS skipped] {e}')
                    num_cluster = 0
            data_status = 1

        # 不管什么 TLV, 都用 tlv_length 精确跳到下一个 TLV 头
        idX = tlv_start + tlv_length

    # ---------- 6. 永远按 totalPacketLen 推进缓冲区 ----------
    shift = totalPacketLen
    if byteBufferLength >= shift:
        byteBuffer[:byteBufferLength - shift] = byteBuffer[shift:byteBufferLength]
        byteBuffer[byteBufferLength - shift:] = 0
        byteBufferLength -= shift
    else:
        byteBufferLength = 0

    if not parse_ok:
        return 0, frameNumber, {}, num_moving_point, num_cluster

    return data_status, frameNumber, det_obj_set, num_moving_point, num_cluster


# ------------------------------------------------------------------

# Funtion to update the data and display in the plot
def update():
    number= 0
    data_status = 0
    global det_obj_set
    global byu
    left_b = Dataport.in_waiting
    x = []
    y = []
    # Read and parse the received data
    data_status, frameNumber, det_obj_set, num_moving_point, num_cluster = readAndParseData68xx(Dataport, configParameters)

    if data_status and len(det_obj_set["x"]) > 0:
        #print(det_obj_set)
        number += num_moving_point
        x = -det_obj_set["x"]
        y = det_obj_set["y"]

    return data_status, number, num_moving_point, num_cluster


# -------------------------    MAIN   -----------------------------------------

# Configurate the serial port
CLIport, Dataport = serialConfig(configFileName)

# Get the configuration parameters from the configuration file
configParameters = parseConfigFile(configFileName)

# Main loop
det_obj_set = {}
frameData = {}
currentIndex = 0
num_moving_point = 0
num_cluster = 0
if __name__ == '__main__':
    while True:
        try:
        # Update the data and check if the data is okay
            data_status = update()

            if data_status:
            # Store the current frame into frameData
                frameData[currentIndex] = det_obj_set
                currentIndex += 1
            time.sleep(0.1)  # Sampling frequency of 30 Hz

    # Stop the program and close everything if Ctrl + c is pressed
        except KeyboardInterrupt:
            CLIport.write(('sensorStop\n').encode())
            CLIport.close()
            Dataport.close()
            win.close()
            break

