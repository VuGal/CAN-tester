from robot.api.deco import keyword
import time
import os
import can
from robot.api import logger
import json


class UnitInterface:
    def testSniff(self, canBus, messageList):
        pass

    def getSetOfSentErrors(self):
        pass

    def testNode(self, error_code):
        pass

    def getUnitErrorCodes(self):
        pass


class ErrorInterface:
    def getCode(self):
        pass

    def getDescription(self):
        pass

    def wasSent(self):
        pass

    def setSent(self, value):
        pass

    def setUnknown(self, value):
        pass

    def getUnknown(self):
        pass

    def isNotError(self):
        pass

    def setNotError(self, value):
        pass


class UnitComposite(UnitInterface):
    def __init__(self):
        self.unitList = []
        self.counter = 0
        self.keywords = Keywords()

    def add(self, unit):
        self.unitList.append(unit)
        self.counter = self.counter + 1

    def getCount(self):
        return self.counter

    def testSniff(self, canBus, messageList):
        self.keywords.set_filters(canBus, 0x0000, 0x0000)
        for i in self.unitList:
            i.testSniff(canBus, messageList)

    def testNode(self, error_code):
        returnList = []
        for i in self.unitList:
            returnList.append(i.testNode(error_code))
        return returnList

    def getUnitErrorCodes(self):
        errorCodesList = []
        for i in self.unitList:
            errorCodesList.append(i.getUnitErrorCodes())
        return errorCodesList

    def getSetOfSentErrors(self):
        sentErrorsList = []
        for i in self.unitList:
            sentErrorsList.append(i.getSetOfSentErrors())
        return sentErrorsList


class Unit(UnitInterface):
    def __init__(self, name, code, errorCodes):
        self.name = name
        self.code = code
        self.errorCodes = errorCodes
        self.keywords = Keywords()

    def testSniff(self, canBus, messageList):
        for msg in messageList:
            local_code = int(msg.data.hex(), base=16)
            if msg.arbitration_id != self.code:
                continue
            if self.errorCodes.isCodeInComposite(local_code) is False and self.errorCodes.getUnknownByCode(
                    local_code) is False:
                self.errorCodes.add(ErrorCode(local_code, 'Unknown', False))
                self.errorCodes.setUnknownByCode(local_code, True)
                self.errorCodes.setSentByCode(local_code, True)
            else:
                self.errorCodes.setSentByCode(local_code, True)

    def getSetOfSentErrors(self):
        return self.errorCodes.generateSetOfSentErrors()

    def testNode(self, error_code):
        if self.errorCodes.isCodeInComposite(error_code) is False:
            return "Unknown Fail"
        if self.errorCodes.isNotErrorByCode(error_code):
            return "Pass"
        return "Fail"

    def getUnitErrorCodes(self):
        return self.errorCodes


class ErrorCodeComposite(ErrorInterface):
    def __init__(self):
        self.errorCodeList = []
        self.count = 0

    def add(self, errorCode: ErrorInterface):
        self.errorCodeList.append(errorCode)
        self.count = self.count + 1
        return True

    def getCode(self):
        codeList = []
        for i in range(len(self.errorCodeList)):
            codeList.append(self.errorCodeList[i].getCode())
        return codeList

    def getCount(self):
        return self.count

    def getUnknown(self):
        unknownList = []
        for i in range(len(self.errorCodeList)):
            unknownList.append(self.errorCodeList[i].getUnknown())
        return unknownList

    def getDescription(self):
        codeList = []
        for i in range(len(self.errorCodeList)):
            codeList.append(self.errorCodeList[i].getDescription())
        return codeList

    def wasSent(self):
        wasSentCheck = False
        for i in range(len(self.errorCodeList)):
            if self.errorCodeList[i].wasSent() is True:
                wasSentCheck = True
        return wasSentCheck

    def setSent(self, value):
        for i in range(len(self.errorCodeList)):
            self.errorCodeList[i].setSent(value)
        return True

    def setUnknown(self, value):
        for i in range(len(self.errorCodeList)):
            self.errorCodeList[i].setUnknown(value)
        return True

    def setNotError(self, value):
        for i in range(len(self.errorCodeList)):
            self.errorCodeList[i].setNotError(value)
        return True

    def setSentByCode(self, code, value):
        for i in self.errorCodeList:
            if i.getCode() == code:
                i.setSent(value)
                return True
        return False

    def setNotErrorByCode(self, code, value):
        for i in self.errorCodeList:
            if i.getCode() == code:
                i.setNotError(value)
                return True
        return False

    def setUnknownByCode(self, code, value):
        for i in self.errorCodeList:
            if i.getCode() == code:
                i.setUnknown(value)
                return True
        return False

    def getUnknownByCode(self, code):
        for i in self.errorCodeList:
            if i.getCode() == code:
                return i.getUnknown()
        return False

    def isNotErrorByCode(self, code):
        for i in self.errorCodeList:
            if i.getCode() == code:
                return i.isNotError()
        return False

    def getDescriptionByCode(self, code):
        for i in self.errorCodeList:
            if i.getCode() == code:
                return i.getDescription()
        return "Description not found"

    def wasSentByCode(self, code):
        for i in self.errorCodeList:
            if i.getCode() == code:
                return i.wasSent()
        return False

    def isCodeInComposite(self, code):
        for i in self.errorCodeList:
            if i.getCode() == code:
                return True
        return False

    def printUnsent(self):
        for i in range(len(self.errorCodeList)):
            if self.errorCodeList[i].wasSent() is False:
                logger.info(f"Error Code {self.errorCodeList[i].getCode()} was not sent")
                logger.info(f"Description: {self.errorCodeList[i].getDescription()}")
        return True

    def generateSetOfSentErrors(self):
        generatedSet = set()
        for i in self.errorCodeList:
            if (i.wasSent()):
                generatedSet.add(hex(i.getCode()))
        return generatedSet


class ErrorCode(ErrorInterface):

    def __init__(self, code, description, isNotErrorVar):
        self.code = code
        self.description = description
        self.wasSentVar = False
        self.isUnknown = False
        self.isNotErrorVar = isNotErrorVar

    def getCode(self):
        return self.code

    def getDescription(self):
        return self.description

    def wasSent(self):
        return self.wasSentVar

    def setSent(self, value):
        self.wasSentVar = value

    def setUnknown(self, value):
        self.isUnknown = value

    def getUnknown(self):
        return self.isUnknown

    def isNotError(self):
        return self.isNotErrorVar

    def setNotError(self, value):
        self.isNotErrorVar = value


class Keywords:
    def __init__(self):
        self.bitRates = [125000, 250000, 500000, 1000000]
        self.can0 = None
        self.bitRate = None
        self.isExtendedId = None

    def _inner_sweep_bitrates(self, bitrate):
        try:
            self._inner_set_can0_up(bitrate)
            self.set_filters(self.can0, 0x0000, 0x0000)
            msg = self.receive_message(self.can0, 10)  # Wait for message for 10 seconds
            if msg is not None:
                self.isExtendedId = msg.is_extended_id
                self.bitRate = bitrate
                return True
            self.inner_set_can0_down()
        except:
            self.inner_set_can0_down()
            raise
        return False

    def _inner_set_can0_up(self, bitrate):
        time.sleep(1)
        try:
            os.system('sudo modprobe -r mcp251x')
            os.system('sudo modprobe mcp251x')
            os.system(f'sudo ip link set can0 up type can bitrate {bitrate}')
            self.can0 = can.interface.Bus(channel='can0', bustype='socketcan', receive_own_messages=False)
        except:
            os.system('sudo ip link set can0 down')
            raise
        return 1

    @staticmethod
    def inner_set_can0_down():
        time.sleep(1)
        os.system('sudo ip link set can0 down')

    @staticmethod
    def set_filters(can_bus, id, mask):
        can_bus.set_filters([{"can_id": id, "can_mask": mask}])

    @staticmethod
    def receive_message(can_bus, timeout):
        msg = can_bus.recv(timeout)
        return msg

    @staticmethod
    def send_message(can_bus, id, is_remote, is_extended):
        message = can.Message(arbitration_id=id, is_remote_frame=is_remote, is_extended_id=is_extended)
        can_bus.send(message)
        time.sleep(0.1)

    @staticmethod
    def fill_unit(mode, unit, data):
        UnitErrors = ErrorCodeComposite()
        for x in data[mode][unit]:
            if x == 'Name' or x == 'No Error Code':
                continue
            if int(data[mode][unit]["No Error Code"], 0) == int(x, 0):
                UnitErrors.add(ErrorCode(int(x, 0), data[mode][unit][x], True))
            else:
                UnitErrors.add(ErrorCode(int(x, 0), data[mode][unit][x], False))

        UnitObj = Unit(data[mode][unit]["Name"], int(unit, 0), UnitErrors)
        return UnitObj

    @keyword(name="Test Node Config Mode Send Message")
    def test_node_config_mode_send_message(self, can0, unit, data, is_extended):
        can0.set_filters([{"can_id": int(unit, 0), "can_mask": 0xFFFF}])
        time.sleep(1)
        UnitObj = self.fill_unit("NodeMode", unit, data)
        msg_clear = self.receive_message(can0, 1)
        while msg_clear is not None:
            msg_clear = self.receive_message(can0, 1)
        self.send_message(can0, int(unit, 0), True, is_extended)
        time.sleep(1)
        msg = self.receive_message(can0, 1)
        if msg is None:
            raise Exception("Didn't receive any message")
        local_code = int(msg.data.hex(), base=16)
        Status = UnitObj.testNode(local_code)
        if (Status == "Fail"):
            raise Exception(f"Error code: 0x{msg.data.hex()}: {data['NodeMode'][unit]['0x' + msg.data.hex()]}")
        if (Status == "Unknown Fail"):
            raise Exception(f"Unknown code: 0x{msg.data.hex()}: {data['NodeMode'][unit]['0x' + msg.data.hex()]}")
        logger.info(f"Code 0x{msg.data.hex()}: {data['NodeMode'][unit]['0x' + msg.data.hex()]}")

    @keyword(name="Initialize CAN")
    def initialize_can(self):
        for i in self.bitRates:
            if self._inner_sweep_bitrates(i) is True:
                return self.can0, self.isExtendedId
        raise Exception("No matching bitrate found")

    @keyword(name="Deinitialize CAN")
    def deinitialize_can(self):
        os.system('sudo ip link set can0 down')
        return 1

    @keyword(name="Test Node Full Scan Mode Send Messages")
    def test_node_full_scan_mode_send_messages(self, can0, is_extended):
        testDict = {}
        if is_extended is True:
            raise Exception("This mode is not available for extended type of frames")

        for i in range(2**11):
            can0.set_filters([{"can_id": i, "can_mask": 0xFFFF}])
            self.send_message(can0, i, True, is_extended)
            msg = self.receive_message(can0, 0.5)
            testDict[hex(i)] = msg
        return testDict

    @keyword(name="Test Node Full Scan Mode Label Code Error")
    def test_node_full_scan_mode_label_code_error(self, id, messages):
        if messages[id] is None:
            raise Exception(f"Got no response from the specified ID=={id}")
        logger.info(f"{id} Message: {messages[id]}")

    @keyword(name="Test Sniffing Config Mode")
    def test_sniffing_config_mode(self, can0, unit, data, messages):
        UnitObj = self.fill_unit('SniffMode', unit, data)
        UnitObj.testSniff(can0, messages)
        return UnitObj.getSetOfSentErrors(), UnitObj

    @keyword(name="Test Sniffing Config Mode Label Code Error")
    def test_sniffing_config_mode_label_code_error(self, errorCode, unitObj):
        if (unitObj.getUnitErrorCodes().getUnknownByCode(int(errorCode, 0))):
            raise Exception(f"Unknown code {errorCode}")
        if (not unitObj.getUnitErrorCodes().isNotErrorByCode(int(errorCode, 0))):
            raise Exception(
                f"Error code {errorCode}: {unitObj.getUnitErrorCodes().getDescriptionByCode(int(errorCode, 0))}")
        logger.info(f"Code {errorCode}: {unitObj.getUnitErrorCodes().getDescriptionByCode(int(errorCode, 0))}")

    @keyword(name="Read Json File")
    def read_json_file(self, path):

        file = open(path)
        data = json.load(file)
        return data

    @keyword(name="Collect Messages")
    def collect_messages(self, can0, timeout):
        can0.set_filters([{"can_id": 0x0000, "can_mask": 0x0000}])
        start = time.time()
        stop = time.time()
        messages = []
        while stop - start <= 30:
            msg = self.receive_message(can0, timeout)
            if msg == None:
                raise Exception(f"Found no message within {timeout} seconds")
            logger.info(f"Received packet: {msg}")
            stop = time.time()
            messages.append(msg)
        return messages

    @keyword(name="Get Timestamp")
    def get_timestamp(self):
        return time.time()
