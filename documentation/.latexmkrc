$pdf_previewer = 'evince';

# To specify PDF output
$pdf_mode = 1;        

# Specify the main file
@default_files = ('main.tex');

# To create all output in the build dir instead of source dir
$pdflatex="pdflatex -interaction=nonstopmode %O %S";
$out_dir = 'build';

