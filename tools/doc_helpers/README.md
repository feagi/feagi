# FEAGI Documentation Tools

This directory contains tools to help manage and fix the FEAGI documentation system.

## Available Tools

### Fix HTML Tables

This script identifies and fixes HTML tables in markdown files by converting them to proper markdown tables:

```bash
# First, install the required dependencies
pip install beautifulsoup4

# Run the script with --dry-run to see what would be changed
./fix_html_tables.py ../../docs --dry-run

# Run the script to actually modify the files
./convert_html_tables.py ../../docs
```

### Fix Broken Links

This script identifies and fixes broken links in markdown files:

```bash
# Run the script to identify broken links
./fix_broken_links.py ../../docs

# Use --root to specify the root directory for resolving relative links
./fix_broken_links.py ../../docs --root ../../
```

### Run Docusaurus

This script runs the Docusaurus development server with proper error handling:

```bash
# Run the script to start the Docusaurus server
./run_docusaurus.sh
```

## Common Documentation Issues

### HTML Formatting Errors

Some markdown files contain HTML tables with formatting errors. These files should be excluded from the Docusaurus build or fixed using the `convert_html_tables.py` script.

### Missing Images

Some markdown files reference images that don't exist. The images should be created or the references should be updated.

### Sidebar References

The sidebar configuration files reference markdown files that don't exist. The sidebar files should be updated to remove these references.

## Running Docusaurus

To run the Docusaurus development server:

1. Navigate to the Docusaurus directory:
   ```bash
   cd website/docs
   ```

2. Start the server:
   ```bash
   npm start
   ```

3. If you encounter port conflicts or other issues, you can specify a different port:
   ```bash
   npm start -- --port 3001
   ```

## Troubleshooting

### Module Alias Plugin

If you encounter errors related to `docusaurus-plugin-module-alias`, make sure this plugin is installed or remove it from the Docusaurus configuration.

### HTML Formatting Errors

If you encounter MDX compilation errors like:

```
Expected a closing tag for `<td>` before the end of `paragraph`
```

You should:
1. Fix the HTML in the markdown file using the `convert_html_tables.py` script
2. Exclude the file from the Docusaurus build in `docusaurus.config.ts`

### Missing Images

If you encounter errors like:

```
Image ../../../docs/assets/bdu-architecture.png used in ../../feagi/bdu/arch-bdu.md not found
```

You should create the missing image file.
