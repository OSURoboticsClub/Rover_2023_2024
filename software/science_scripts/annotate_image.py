from PIL import Image, ImageDraw, ImageFont
import os
import sys
image_dir = sys.argv[1] + "/pano_0.jpg"
image = Image.open(image_dir)
draw = ImageDraw.Draw(image)
altitude = "Altitude: "
with open ('altitude.txt','r') as file:
    altitude+= file.read()
   
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",30)
text_position = (180,50)
arrow_start = (320,180)
arrow_end = (320,280)
arrow_color = "Red"
arrow_width = 5
draw.text(text_position, altitude, font=font, fill = "red")

draw.line([arrow_start,arrow_end], fill=arrow_color, width=arrow_width)
draw.polygon([(arrow_start[0] -10, arrow_start[1] + 10),(arrow_start[0] +10, arrow_start[1] + 10), (arrow_start[0], arrow_start[1] - 20)], fill=arrow_color)

image.save(image_dir)
