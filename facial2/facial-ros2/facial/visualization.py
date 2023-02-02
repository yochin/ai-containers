from PIL import Image
import numpy as np
import PIL.ImageDraw as ImageDraw
import PIL.ImageFont as ImageFont
import random

TEXT_COLOR = 'Black'

# Font to draw text on image
FONT_NAME = 'Ubuntu-R.ttf'

# Bounding box colors
COLORS = ['Green',
          'Red', 'Pink',
          'Olive', 'Brown', 'Gray',
          'Cyan', 'Orange']

trk_color = {}

def get_color(trk_id):
    if trk_id not in trk_color:
        color_idx = random.randint(0, len(COLORS) - 1)
        color = COLORS[color_idx]
        trk_color[trk_id] = color
    return trk_color[trk_id]

class ObjectResult:
    """
    Represents a detection result, containing the object label,
    score confidence, and bounding box coordinates.
    """

    def __init__(self, trk_id, label, score, box):
        self.trk_id = trk_id
        self.label = label
        self.score = score
        self.box = box
    
    def __repr__(self):
        #return '{0} ({1}%)'.format(self.label, int(100 * self.score))
        return '{0}'.format(self.label)

def draw_labeled_boxes(image_np, results, min_score=.4):
    """
    Draws labeled boxes according to results on the given image.
    :param image_np: numpy array image
    :param results: list of ObjectResult
    :param min_score: optional min score threshold, default is 40%
    :return: numpy array image with labeled boxes drawn
    """
    results.sort(key=lambda x: x.score, reverse=False)
    image_np_copy = image_np.copy()
    for r in results:
        if r.score >= min_score:
            color = get_color(r.trk_id)

            image_pil = Image.fromarray(np.uint8(image_np_copy)).convert('RGB')
            draw_bounding_box_on_image(image_pil, r.box, color, str(r))
            np.copyto(image_np_copy, np.array(image_pil))

    return image_np_copy


def get_suitable_font_for_text(text, img_width, font_name, img_fraction=0.2):
    """
    Calculates a suitable font for the image given the text and fraction.
    :param text: text that will be drawn
    :param img_width: width of the image
    :param font_name: name of the font
    :param img_fraction: optional desired image fraction allowed for the text 
    :return: suitable font
    """
    fontsize = 1
    font = ImageFont.truetype(FONT_NAME, fontsize)
    while font.getsize(text)[0] < img_fraction*img_width:
        fontsize += 1
        font = ImageFont.truetype(font_name, fontsize)
    return font


def draw_bounding_box_on_image(image, box, color, box_label):
    """
    Draws the box and label on the given image.
    :param image: PIL image
    :param box: numpy array containing the bounding box information
                [top, left, bottom, right]
    :param color: bounding box color
    :param box_label: bounding box label
    """
    im_width, im_height = image.size
    left, top, right, bottom = box

    # Normalize coordinates
    left = max(0, left)
    right = min(right, im_width)
    top = max(0, top)
    bottom = min(bottom, im_height)

    # Draw the detected bounding box
    line_width = int(max(im_width, im_height) * 0.005)
    draw = ImageDraw.Draw(image)
    draw.rectangle(((left, top), (right, bottom)),
                   width=line_width,
                   outline=color)

    # Get a suitable font (in terms of size with respect to the image)
    font = get_suitable_font_for_text(box_label, im_width, FONT_NAME)
    text_width, text_height = font.getsize(box_label)

    # Draw the box label rectangle
    text_bottom = top + text_height
    text_rect = ((left, top),
                 (left + text_width + 2 * line_width,
                  text_bottom + 2 * line_width))
    draw.rectangle(text_rect, fill=color)

    # Draw the box label text
    # right below the upper-left horizontal line of the bounding box
    text_position = (left + line_width, top + line_width)
    draw.text(text_position, box_label, fill=TEXT_COLOR, font=font)
