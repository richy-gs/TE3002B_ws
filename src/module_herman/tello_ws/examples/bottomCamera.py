import cv2
from djitellopy import tello

def run_bottom_video(drone):
    while True:
        # grab frame from the drone’s downward camera
        frame = drone.get_frame_read().frame
        
        # crop to the top-left 320×240 region, then rotate
        crop_img = frame[0:240, 0:320]
        rotated = cv2.rotate(crop_img, cv2.ROTATE_90_CLOCKWISE)
        
        # get its height and width
        h, w = rotated.shape[:2]
        
        # resize to double the dimensions (width, height) as a tuple
        resized = cv2.resize(rotated, (w * 2, h * 2))
        
        # display
        cv2.imshow("Bottom Camera", resized)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


def main():
    drone = tello.Tello()
    drone.connect()
    drone.set_video_direction(drone.CAMERA_DOWNWARD)
    drone.streamon()
    run_bottom_video(drone)


if __name__== "__main__":
    main()