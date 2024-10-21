HOWTO make spinner with new picture:

This is not optimized but:
- Add new picture into selfdrive/assets/
- Make new x_spinner.cc and -.h files in selfdrive/common/
- Modify those so that the new pic file name correspond with these: 
  _binary_NEW_PICTURE_NAME_png_start
  _binary_NEW_PICTURE_NAME_png_end
- Modify the spinner.cc in selfdrive/ui/your_new_spinner_folder/ to refer to the new common/spinner_files
- Modify the Makefile in selfdrive/ui/your_new_spinner_folder/ to comply with these changes
