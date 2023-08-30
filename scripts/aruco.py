import cv2
import numpy as np

from reportlab.lib.pagesizes import letter
from reportlab.platypus import SimpleDocTemplate, Image
 
markersX = 1           #X轴上标记的数量
markersY = 1           #EY轴上标记的数量   本例生成2行6列的格子
markerLength = 100#标记的长度，单位是像素
markerSeparation = 20#每个标记之间的间隔，单位像素
margins = markerSeparation #标记与边界之间的间隔
borderBits = 10 #标记的边界所占的bit位数
showImage = True
 
 
width = markersX * (markerLength + markerSeparation) - markerSeparation + 2 * margins
height =markersY * (markerLength + markerSeparation) - markerSeparation + 2 * margins
 
 
dictionary = cv2.aruco.Dictionary_get( cv2.aruco.DICT_4X4_250)
board = cv2.aruco.GridBoard_create(markersX, markersY, float(markerLength),float(markerSeparation), dictionary)
print(cv2.aruco_GridBoard.getGridSize(board))
#根据aruco的宽度自动生成合适的图片尺寸
img= cv2.aruco_GridBoard.draw(board,(width,height),1)
cv2.imwrite('frame.png', img)

png_file = "/home/joy/OCTOMAP_WS/src/depth2octomap/scripts/frame.png"
# png_file = "/home/joy/Downloads/aruco.png"

pdf_file = "/home/joy/OCTOMAP_WS/src/depth2octomap/scripts/frame.pdf"
doc = SimpleDocTemplate(pdf_file, pagesize=letter)
img = Image(png_file, width=letter[0], height=letter[1])
doc.build([img])


