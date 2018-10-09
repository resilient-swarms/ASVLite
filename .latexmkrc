$dvi_previewer = 'start xdvi -watchfile 1.5';
$ps_previewer  = 'start gv --watch';
$pdf_previewer = 'start evince';

# To specify PDF output
$pdf_mode = 1;        

# Specify the main file
@default_files = ('main.tex');

# To create all output in the build dir instead of source dir
$pdflatex="pdflatex -interaction=nonstopmode %O %S";
$out_dir = 'build';