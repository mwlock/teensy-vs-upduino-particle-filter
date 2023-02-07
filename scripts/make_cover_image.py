from pdf2image import convert_from_path
from PIL import Image, ImageDraw, ImageOps

def pdf_to_image(pdf_file, overlap_percentage=50, shadow_width=10):
    # Convert the PDF to a list of images
    pages = convert_from_path(pdf_file)

    # Calculate the amount of overlap in pixels
    overlap = int(pages[0].size[0] * overlap_percentage / 100)

    # Create a new image with the same height as the first page
    result = Image.new("RGB", (pages[0].size[0] * len(pages) + shadow_width * (len(pages) - 1)*0 - overlap*(len(pages)-1), pages[0].size[1]))

    # Add a black outline and shadow to each page
    for i, page in enumerate(pages[::-1]):
        outline_image = Image.new("RGB", page.size, (0, 0, 0))
        outline_draw = ImageDraw.Draw(outline_image)
        outline_draw.rectangle([(0, 0), (page.size[0] - 1, page.size[1] - 1)], outline=(255, 255, 255), width=100)
        shadow = ImageOps.expand(outline_image, border=shadow_width, fill=(200, 200, 200))
        shadow_offset = (len(pages) - i - 1) * (pages[0].size[0] - overlap)
        result.paste(shadow, (shadow_offset + shadow_width, shadow_width))
        outline_image.paste(page, (0, 0))
        page = outline_image
        
        # Paste each page on top of the previous one, offsetting it to the right
        result.paste(page, (shadow_offset, 0))

    return result

# Example usage
pdf_file = "report.pdf"
result = pdf_to_image(pdf_file, overlap_percentage=80, shadow_width=10)
result.save("output.png")