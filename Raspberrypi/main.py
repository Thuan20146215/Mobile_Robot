import cv2
import numpy as np
import time
from picamera2 import Picamera2
import line_detect  # Assuming this is a custom module for line detection
import serial
import threading
from pyzbar.pyzbar import decode
qr_save = None
select = None
#warn = 2
################
turn = False
wait = False
################
ser_curve = '/dev/ttyACM0' ## port sent error
ser_QR = '/dev/ttyUSB0' ## port sent a string
a = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5) ## port sent a string
#b = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.5)
######
inputxt = 'embedded/DATN/linefl/output.txt' ## input txt file
outputxt = 'embedded/DATN/linefl/path.txt' ## output txt file
######

class CurveSender:
    def __init__(self, serial_port):
        self.serial_port = serial.Serial(serial_port, 115200, timeout=0.5)
        self.curve_value = 0
        self.curve_lock = threading.Lock()
        self.curve_thread = threading.Thread(target=self.send_curve_periodically, daemon=True)
        self.curve_thread.start()

    def send_curve_periodically(self):
        count = 0
        global turn
        global wait
        while True:
            time.sleep(0.2)
            with self.curve_lock:
                #print(self.curve_value)
                self.serial_port.write(str(self.curve_value).encode())
                #a.write("F".encode())
                if select:
                    #a.write("F".encode())
                    if turn and not wait:
                        a.write("A".encode())
                        wait = True
                        print("RObot is Turning around........")
                        print(wait)
                    if wait:
                        # response = a.read(3)
                        # print("rec")
                        if a.in_waiting > 0:
                            response = a.read(3)
                            print("rec")
                            if response == b'D\r\n':
                                turn = False
                                print(response)
                                wait = False
                    if not turn and not wait:
                        a.write("F".encode())
                        #print("sent")
                        # self.serial_port.write(str(self.curve_value).encode())
                        #print(self.curve_value)
    def update_curve(self, curve):
        with self.curve_lock:
            self.curve_value = curve

class QRProcessor:
    def __init__(self, serial_port):
        self.serial_port = serial.Serial(serial_port, 115200, timeout=0.5)

    def read_qr_code(self, img):
        # imgHsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        for barcode in decode(img):
            data = barcode.data.decode('utf-8')
            return data
        return None

    def send_message(self, message):
        self.serial_port.write(message.encode())

class FileProcessor:
    def __init__(self, filename):
        self.filename = filename
        self.total = []
        self.cid = []
        self.info = []
        self.read_data_from_file()
        self.output_data = []

    def read_data_from_file(self):
        try:
            with open(self.filename, 'r') as f:
                data = [list(map(int, line.strip().split())) for line in f]
            self.total = data.copy()
            self.info = data.pop(0) if data else []
            data_excluding_last = data[:2] if len(data) > 1 else []
            self.cid = np.array(data_excluding_last).T.tolist() if data_excluding_last else []
        except FileNotFoundError:
            print("File not found.")
        except Exception as e:
            print(f"An error occurred: {e}")

    ####
    def read_output_data_from_file(self, output_filename):
        try:
            with open(output_filename, 'r') as f:
                self.output_data = [list(map(int, line.strip().split())) for line in f]
        except FileNotFoundError:
            print("Output file not found.")
        except Exception as e:
            print(f"An error occurred: {e}")
    ####
    def check_condition(self):
        self.read_data_from_file()
        if self.total and len(self.total) >= 4:
            if self.total[4][0] == 1:
                return True
        return False

    def update_file(self, filename, data):
        try:
            with open(filename, 'w') as f:
                if isinstance(data, list):
                    for row in data:
                        f.write(' '.join(map(str, row)) + '\n')
                elif isinstance(data, int):
                    f.write(str(data) + '\n')
                else:
                    print("Unsupported data type for writing to the file.")
        except Exception as e:
            print(f"An error occurred while updating the file: {e}")

class LineDetector:
    def __init__(self, avg_val=10):
        self.curve_list = []
        self.avg_val = avg_val
        self.actions = {
            3: "F",
            1: "L",
            2: "R",
        }

    def choose_contour(self, contours, image_width=480, selection=2):
        center_x = image_width // 2
        min_distance = float('inf')
        best_contour = None
        sorted_contours = sorted(contours, key=lambda cnt: cnt[0])
        direct = self.actions.get(selection, "Invalid selection")
        #print(f"dir: {direct}")
        if direct == "Invalid selection":
            #print("Error: Invalid selection value.")
            return None

        if len(sorted_contours) == 3:
            if direct == 'L':
                best_contour = sorted_contours[0]
                #a.write("L".encode())
            elif direct == 'F':
                best_contour = sorted_contours[1]
            elif direct == 'R':
                best_contour = sorted_contours[2]
                #a.write("R".encode())
        elif len(sorted_contours) == 2:
            if direct == 'L':
                #a.write("L".encode())
                best_contour = sorted_contours[0]
            elif direct == 'R':
                best_contour = sorted_contours[1]
                #a.write("R".encode())
        else:
            for cnt in contours:
                x, _, w, _ = cnt
                contour_center_x = x + w // 2
                distance = abs(contour_center_x - center_x)
                if distance < min_distance:
                    min_distance = distance
                    best_contour = cnt
        return best_contour

    def get_lane_curve(self, img, display=2, selection=2):
        img_copy = img.copy()
        img_direct = img.copy()
        img_direct_warp = img.copy()
        img_result = img.copy()

        img_thres = line_detect.thresholding(img)
        hT, wT, _ = img.shape
        points = line_detect.valTrackbars()
        img_warp = line_detect.warpImg(img_thres, points, wT, hT)
        img_warp_points = line_detect.drawPoints(img_copy, points)
        img_inv_warp = line_detect.warpImg(img_warp, points, wT, hT, inv=True)

        bounding_boxes, img_contours = line_detect.find_large_contours(img_inv_warp, img_copy)
        bounding_box = None
        if bounding_boxes:
            bounding_box = self.choose_contour(bounding_boxes, wT, selection)

        curve_average_point = 0
        if bounding_box is not None:
            x, y, w, h = bounding_box
            cv2.rectangle(img_direct, (x, y), (x + w, y + h), (255, 0, 0), 2)
            points1 = np.float32([(x, y), (x + w, y), (x, y + h), (x + w, y + h)])
            img_direct_warp = line_detect.warpImg(img_thres, points1, wT, hT)
            curve_average_point, img_hist = line_detect.getHistogram(img_direct_warp, display=True, minPer=0.9)
            curve_average_point = round((curve_average_point * w / 480) + x)
        else:
            if select == 1:
                curve_average_point = 40
            elif select == 2: 
                curve_average_point = 450
            else:
                curve_average_point, img_hist = line_detect.getHistogram(img_warp, display=True, minPer=0.9)

        middle_point = 240
        curve_raw = curve_average_point - middle_point

        if curve_average_point >= 0:
            cv2.circle(img_result, (curve_average_point, img.shape[0]), 20, (0, 255, 255), cv2.FILLED)
        cv2.circle(img_result, (middle_point, img.shape[0]), 20, (255, 0, 0), cv2.FILLED)

        self.curve_list.append(curve_raw)
        if len(self.curve_list) > self.avg_val:
            self.curve_list.pop(0)
        curve = int(sum(self.curve_list) / len(self.curve_list))
        curve = curve//10
        if display != 0:
            img_inv_warp = cv2.cvtColor(img_inv_warp, cv2.COLOR_GRAY2BGR)
            img_inv_warp[0:hT // 3, 0:wT] = 0, 0, 0
            img_lane_color = np.zeros_like(img)
            img_lane_color[:] = 0, 255, 0
            img_lane_color = cv2.bitwise_and(img_inv_warp, img_lane_color)
            img_result = cv2.addWeighted(img_result, 1, img_lane_color, 1, 0)
            midY = 450
            cv2.putText(img_result, str(curve), (wT // 2 - 80, 85), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 0, 255), 3)
            cv2.line(img_result, (wT // 2, midY), (wT // 2 + (curve * 3), midY), (255, 0, 255), 5)
            cv2.line(img_result, ((wT // 2 + (curve * 3)), midY - 25), (wT // 2 + (curve * 3), midY + 25), (0, 255, 0), 5)

            for x in range(-30, 30):
                w = wT // 20
                cv2.line(img_result, (w * x + int(curve // 50), midY - 10), (w * x + int(curve // 50), midY + 10), (0, 0, 255), 2)
        if display == 2:
            img_stacked = line_detect.stackImages(0.7, ([img, img_inv_warp, img_direct_warp], [img_direct, img_contours, img_result]))
            cv2.imshow('ImageStack', img_stacked)
        elif display == 1:
            cv2.imshow('Result', img_direct)

        return curve

class CameraHandler:
    def __init__(self):
        self.picam2 = self.init_picamera()

    def init_picamera(self):
        picam2 = Picamera2()
        picam2.preview_configuration.main.size = (640, 360)
        picam2.preview_configuration.main.format = "RGB888"
        picam2.preview_configuration.align()
        picam2.configure("preview")
        picam2.start()
        time.sleep(0.1)
        return picam2

    def capture_image(self):
        return self.picam2.capture_array()
    
# def obstacleDetect(value):
#     if int(value):
#         if 30 <= int(value) <= 90:
#             #a.write("T".encode())
#             return 1
#         elif int(value) < 30:
#             #a.write("S".encode())
#             return 0
#         #else:
#             #a.write("F".encode())
#     return 2

def process_qr_code(qr_code, i, processor, qr_processor):
    # select = None
    global qr_save
    global select
    global turn
    turn = False
    if qr_code:
        if isinstance(qr_code, str):
            processor.output_data[0][0] = int(qr_code)
            processor.update_file(outputxt, processor.output_data)
        if int(qr_code) == processor.info[1]: ###stop in a last station
            print("done")
            a.write("S".encode())
            print("STOPPPP")
            print("##############")
            processor.total[4][0] = 0  # row 5, operation state 
            processor.update_file(inputxt,processor.total)

            processor.output_data[1][0] = 0 #stop
            processor.update_file(outputxt, processor.output_data)
            select = None
            return i, select, True
        elif i > len(processor.cid)-1:  ###stop when the last station is wrong
            print("error1")

            qr_processor.send_message('S')
            processor.total[4][0] = 0  # row 5, operation state
            processor.update_file(inputxt,processor.total)
            a.write("S".encode())
            print("STOPPPP")
            print("##############")

            processor.output_data[1][0] = 2 #error
            processor.update_file(outputxt, processor.output_data)
            select = None
            return i, select, True
        elif int(qr_code) == processor.cid[i][0]: 
            select = processor.cid[i][1]
            print(f"direct: {processor.cid[i][1]}")

            processor.output_data[1][0] = 1 #running
            processor.update_file(outputxt, processor.output_data)
            if processor.total[3][0] == 1:  # row 4, turn around state
                print("Turn around")
                turn = True 
                processor.total[3][0] = 0
                processor.update_file(inputxt,processor.total)
            
            i += 1
            qr_save = qr_code
            return i, select, False
        else:
            print("error2")
            qr_processor.send_message('S')
            processor.total[4][0] = 0  # row 5, operation state
            processor.update_file(inputxt,processor.total)
            a.write("S".encode())
            print("STOPPPP")
            print("##############")

            processor.output_data[1][0] = 2 #error
            processor.update_file(outputxt, processor.output_data)
            select = None
            return i, select, True
    return i, select, False

def main():
    global warn
    global select
    intialTrackBarVals = [45, 112, 4, 210 ]#[60, 35, 20, 114 ]#[100, 40, 20, 154 ]
    line_detect.initializeTrackbars(intialTrackBarVals)###intial
    camera = CameraHandler()
    curve_sender = CurveSender(ser_curve)
    qr_processor = QRProcessor(ser_QR)

    processor = FileProcessor(inputxt)
    processor.read_output_data_from_file(outputxt)
    #distance_receiver = DistanceReceiver(b, processor)
    cap = cv2.VideoCapture(0)
    line_detector = LineDetector()
    i = 0
    a.write("S".encode())
    while True:
        print("waiting the loop...")
        while not processor.check_condition():
            pass
        time.sleep(1)
        print(f"1: {processor.total}")
        print(f"2: {processor.cid}")
        print(f"3: {processor.info}")
        print(f"4: {processor.total[3][0]}")
        print("Condition met. Starting the loop...")
        #a.write("F".encode())
        while True:
            #success, img = cap.read()
            img = camera.capture_image()
            img = cv2.resize(img, (480, 240))
            qr_code = qr_processor.read_qr_code(img)

            # time.sleep(5)
            #print("camera opens")
            ########
            if not qr_save == qr_code:
                i, select, should_break = process_qr_code(qr_code, i, processor, qr_processor)
                #print("select: ",select)
                if should_break:
                    break
            ##########
            #select = processor.output_data[0][0]
            curve = line_detector.get_lane_curve(img, display=2, selection=select)
            curve_sender.update_curve(curve)
            #########
            
            if cv2.waitKey(1) == ord('q'):
                #a.write("S".encode())
                break
        time.sleep(1)
        select = None

if __name__ == '__main__':
    main()
