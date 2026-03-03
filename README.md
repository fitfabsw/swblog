# CPD 軟體部落

A Hugo-based blog using the [PaperMod](https://github.com/adityatelange/hugo-PaperMod) theme.

## Prerequisites

- **Hugo Extended** >= 0.146.0  
  The theme uses SCSS and requires the Extended edition.

- **Go** >= 1.18  
  Used for Hugo modules (theme dependency).

### Installing Hugo Extended

**macOS (Homebrew):**
```bash
brew install hugo
```

**Linux:**
```bash
# Download from https://gohugo.io/installation/linux/
# Or use your package manager, e.g.:
sudo apt install hugo  # Ubuntu/Debian (ensure it's Extended)
```

**Windows:**
Download the `hugo_extended` archive from [Hugo Releases](https://github.com/gohugoio/hugo/releases) and add it to your PATH.

Verify installation:
```bash
hugo version
# Should show "extended" in the output
```

## Installation

1. **Clone the repository:**
   ```bash
   git clone <repo-url> swblog
   cd swblog
   ```

2. **Install npm dependencies** (for Netlify CMS admin UI):
   ```bash
   npm install
   ```

## Running the App

**Development server** (with live reload):
```bash
hugo server --disableFastRender
```

Or using npm:
```bash
npm run dev
```

Open http://localhost:1313 in your browser.

**Build for production:**
```bash
hugo --minify
```

Or:
```bash
npm run build
```

Output is written to the `public/` directory.

## Production

The build command only generates static files; it does not start a web server. To run the production version locally or serve the built site:

**1. Build the site** (with base URL if deploying to a subpath):
```bash
hugo --minify --baseURL "http://your-domain/swblog/"
```

**2. Serve the `public/` directory** using one of these options:

- **Quick local preview** (Python):
  ```bash
  cd public && python3 -m http.server 8080
  ```
  Open http://localhost:8080

- **Production web server** (e.g. Nginx): Point the document root to the `public/` directory (or copy its contents to your web root, as in the deploy workflow).
