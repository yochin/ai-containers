from datetime import datetime
import rtsp
import numpy as np
import os
import cv2

if __name__ == '__main__':
    ip_addr_1 = 'rtsp://admin:1234567@192.168.50.146:554/stream1'
    ip_addr_2 = 'rtsp://admin:1234567@192.168.50.30:554/stream1'
    ip_addr_3 = 'rtsp://admin:1234567@192.168.50.3:554/stream1'

    cap1 = rtsp.Client(rtsp_server_uri=ip_addr_1, verbose=True)

    save_folder1 = '/Users/zebehn/Downloads/'

    while True:
        frame1 = cap1.read()

        if frame1 is not None:
            # frame_resized = frame1.resize((int(frame1.width / 4), int(frame1.height / 4)))
            frame_resized = frame1
            cv_im = np.array(frame_resized)
            cv_im_bgr = cv_im[:, :, ::-1].copy()  # rgb to bgr

            # get current time
            dt = datetime.now()
            cur_time = dt.strftime('%Y-%m-%d-%H-%M-%S-%f')
            print(cur_time)

            # # save image
            #strfilename = os.path.join(save_folder1, 'captured_%s.jpg' % cur_time) # jpeg only
            #frame1.save(strfilename, 'JPEG')

            cv2.imshow("img", cv_im_bgr)
            cv2.waitKey(1)
