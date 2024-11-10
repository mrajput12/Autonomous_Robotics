import os
import numpy
from PIL import Image, ImageDraw
label_path = "/media/souz/disk/OneDrive - uni-osnabrueck.de/UTP/Done-Inventorymgt/dev/AI_pipeline/yolov7-segmentation/runs/predict-seg/exp/labels"
image_path = "/media/souz/disk/OneDrive - uni-osnabrueck.de/UTP/Done-Inventorymgt/dev/AI_pipeline/yolov7-segmentation/runs/predict-seg/exp"
list_dir = os.listdir(label_path)

print(list_dir)
for file in list_dir:
    file_path = os.path.join(label_path, file)
    print('this is file path', file_path)
    base_name = os.path.basename(file.split(".")[0])
    single_image_path = os.path.join(image_path,base_name+".JPEG")
    save_img_path = os.path.join(image_path, "extracted_masks_via_ploygn", base_name + ".png")

    if base_name+'.png' not in os.listdir(image_path+"/extracted_masks_via_ploygn"):
        print(single_image_path)
        with open(file_path, "r") as f:
            lines = f.read().replace("\n", " ")
            lines=list(map(int, lines.split()))

            #lines = lines.strip()
            #lines = lines.split(" ")
            print('this is type of line', type(lines[0]))
            lines=lines[1:]
            print('Total number of elements', len(lines))
            print(lines)
            #print('here problem liess',lines[2614])
            my_polygons = [(lines[i], lines[i + 1]) for i in range(0, len(lines), 2)]
            print(my_polygons)


            # cutting images based on given polygons and save them
            # read image as RGB and add alpha (transparency)
            im = Image.open(single_image_path).convert("RGBA")

            # convert to numpy (for convenience)
            imArray = numpy.asarray(im)

            # create mask
            polygon = my_polygons
            maskIm = Image.new('L', (imArray.shape[1], imArray.shape[0]), 0)
            ImageDraw.Draw(maskIm).polygon(polygon, outline=1, fill=1)
            mask = numpy.array(maskIm)

            # assemble new image (uint8: 0-255)
            newImArray = numpy.empty(imArray.shape,dtype='uint8')

            # colors (three first columns, RGB)
            newImArray[:,:,:3] = imArray[:,:,:3]

            # transparency (4th column)
            newImArray[:,:,3] = mask*255

            # back to Image from numpy
            newIm = Image.fromarray(newImArray, "RGBA")

            print(save_img_path)
            newIm.save(save_img_path)