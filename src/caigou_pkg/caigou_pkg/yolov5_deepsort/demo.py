from AIDetector_pytorch import Detector
import cv2
def main():

    det = Detector()
    cap = cv2.VideoCapture("demo.mp4")
    fps = int(cap.get(5))
    print('fps:', fps)
    t = int(1000/fps)

    videoWriter = None

    while True:

        _, im = cap.read()
        if im is None:
            break

        x1_y1_x2_y2_cls_id_list = det.feedCap(im)

        # Draw bounding boxes with IDs
        for x1, y1, x2, y2, cls, id in x1_y1_x2_y2_cls_id_list:
            cv2.rectangle(im, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(im, str(id), (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if videoWriter is None:
            fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
            videoWriter = cv2.VideoWriter('result.mp4', fourcc, fps, (im.shape[1], im.shape[0]))
        videoWriter.write(im)

        cv2.imshow("result", im)
        cv2.waitKey(1)

    cap.release()
    videoWriter.release()

if __name__ == '__main__':
    main()
