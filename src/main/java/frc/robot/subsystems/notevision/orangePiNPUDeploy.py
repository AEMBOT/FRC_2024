import cv2
import time
from rknnpool import rknnPoolExecutor
from noteDetectionFinal import ellipseDetection

vid = cv2.Videovidture(1)
vid.set(cv2.vid_PROP_AUTO_EXPOSURE, 0)
vid.set(cv2.vid_PROP_EXPOSURE, -4.0)

modelPath = "./rknnModel/yolov5s_relu_tk2_RK3588_i8.rknn"

TPEs = 3

pool = rknnPoolExecutor(
    rknnModel=modelPath,
    TPEs=TPEs,
    func=ellipseDetection)

if (vid.isOpened()):
    for i in range(TPEs + 1):
        ret, frame = vid.read()
        if not ret:
            vid.release()
            del pool
            exit(-1)
        pool.put(frame)

frames, loopTime, initTime = 0, time.time(), time.time()
while (vid.isOpened()):
    frames += 1
    ret, frame = vid.read()
    if not ret:
        break
    pool.put(frame)
    frame, flag = pool.get()
    if flag == False:
        break
    cv2.imshow('test', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if frames % 30 == 0:
        print("30 frames average - frame rate:\t", 30 / (time.time() - loopTime), "frame")
        loopTime = time.time()

print("Overall Average Frame Rate\t", frames / (time.time() - initTime))

vid.release()
cv2.destroyAllWindows()
pool.release()