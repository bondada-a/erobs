# Latexmk configuration for organized builds
# All auxiliary files go to build/ subdirectory

# Output directory for auxiliary files
$aux_dir = 'build';
$out_dir = 'build';

# Use pdflatex by default
$pdf_mode = 1;
$pdflatex = 'pdflatex -interaction=nonstopmode -synctex=1 %O %S';

# Clean up extra extensions
$clean_ext = 'synctex.gz synctex.gz(busy) run.xml tex.bak bbl bcf fdb_latexmk run tdo %R-blx.bib';

# Ensure build directory exists
system("mkdir -p $aux_dir");
