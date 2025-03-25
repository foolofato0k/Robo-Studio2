from pixels2svg import pixels2svg

if __name__ == '__main__':
    input_path = 'test_images/poster.png'
    output_path = 'test_images/svg_out.svg'

    pixels2svg(input_path, output_path, remove_background=True)