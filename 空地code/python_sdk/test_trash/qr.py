import cv2
from pyzbar.pyzbar import decode


cap = cv2.VideoCapture(0)
# qrCodeDetector = cv2.QRCodeDetector()


def decode_qr_code(image):
    #image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    decoded_objects = decode(image)
    
    height, width = image.shape[:2]
    center_x = width // 2
    center_y = height // 2

    min_distance = float('inf')
    nearest_qr = None

    for obj in decoded_objects:
        qr_center_x = obj.rect.left + obj.rect.width // 2
        qr_center_y = obj.rect.top + obj.rect.height // 2

        distance = ((qr_center_x - center_x) ** 2 + (qr_center_y - center_y) ** 2) ** 0.5

        if distance < min_distance:
            min_distance = distance
            nearest_qr = obj

    if nearest_qr is not None:
        # Draw a rectangle around the QR code
        cv2.rectangle(image, (nearest_qr.rect.left, nearest_qr.rect.top), 
                      (nearest_qr.rect.left + nearest_qr.rect.width, nearest_qr.rect.top + nearest_qr.rect.height), 
                      (0, 255, 0), 2)
        
        # Put the decoded text above the rectangle
        cv2.putText(image, nearest_qr.data.decode('utf-8'), 
                    (nearest_qr.rect.left, nearest_qr.rect.top - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Show the image
    cv2.imshow("QR Code", image)
    if nearest_qr is not None:
        return nearest_qr.data.decode('utf-8')
    else:
        return None
    
while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera not found")
        break

    decoded_info = decode_qr_code(frame)

    print(decoded_info)

    if cv2.waitKey(1) == ord('q'):
        break

# qrcode_image = cv2.imread("C:\\Users\\29943\\Desktop\\3.png")



# qrCodeDetector = cv2.QRCodeDetector()
# data, bbox, straight_qrcode = qrCodeDetector.detectAndDecode(qrcode_image)

# print(data)
