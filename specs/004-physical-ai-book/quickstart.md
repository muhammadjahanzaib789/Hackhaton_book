# Quickstart: Physical AI & Humanoid Robotics Book

**Branch**: `004-physical-ai-book` | **Date**: 2025-12-15
**Purpose**: Get the documentation site running locally in under 10 minutes

## Prerequisites

Before you begin, ensure you have:

- [ ] Node.js 18+ installed (`node --version`)
- [ ] npm 9+ installed (`npm --version`)
- [ ] Git installed (`git --version`)
- [ ] A code editor (VS Code recommended)

## Quick Setup

### 1. Clone the Repository

```bash
git clone https://github.com/[username]/physical-ai-book.git
cd physical-ai-book
```

### 2. Install Dependencies

```bash
npm install
```

**Expected Output**:
```
added XXX packages in XXs
```

### 3. Start Development Server

```bash
npm start
```

**Expected Output**:
```
[SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

### 4. Open in Browser

Navigate to `http://localhost:3000` to see the book site.

## Verification Checklist

- [ ] Homepage loads without errors
- [ ] Sidebar navigation shows all modules
- [ ] Code syntax highlighting works
- [ ] Search functionality available

## Project Structure

```text
physical-ai-book/
├── docs/                    # Markdown lesson content
│   ├── intro.md             # Book introduction
│   ├── module-01-ros2/      # ROS 2 module
│   ├── module-02-digital-twin/
│   ├── module-03-isaac/
│   ├── module-04-vla/
│   └── module-05-capstone/
├── static/img/              # Images and diagrams
├── src/
│   ├── components/          # React components
│   └── pages/               # Custom pages
├── code-examples/           # Runnable code (not in docs)
├── docusaurus.config.js     # Site configuration
├── sidebars.js              # Navigation structure
└── package.json             # Dependencies
```

## Common Commands

| Command | Description |
|---------|-------------|
| `npm start` | Start development server |
| `npm run build` | Build production site |
| `npm run serve` | Serve production build locally |
| `npm run clear` | Clear build cache |

## Creating New Content

### Add a New Lesson

1. Create file: `docs/module-NN-slug/lesson-NN-title.md`

2. Add frontmatter:
```yaml
---
sidebar_position: N
title: "Lesson Title"
description: "Brief description"
---
```

3. Follow the [Module Contract](./contracts/module-contract.md) structure

### Add a Code Example

1. Create file: `code-examples/module-nn/example-name.py`

2. Include required documentation:
```python
"""
Problem Solved: [description]
Assumptions: [list]
Failure Modes: [list]
"""
```

3. Reference in lesson with syntax highlighting:
````markdown
```python title="example-name.py"
[code here]
```
````

## Deployment

### Build for Production

```bash
npm run build
```

**Output**: Static files in `build/` directory

### Deploy to GitHub Pages

Automatic deployment via GitHub Actions on push to `main` branch.

Manual deployment:
```bash
GIT_USER=[username] npm run deploy
```

## Troubleshooting

### Issue: npm install fails

**Diagnosis**:
```bash
node --version  # Must be 18+
npm cache clean --force
```

**Resolution**: Update Node.js or clear npm cache

### Issue: Port 3000 already in use

**Resolution**:
```bash
npm start -- --port 3001
```

### Issue: Build fails with MDX error

**Diagnosis**: Check for unclosed JSX tags in Markdown files

**Resolution**: Ensure all JSX components are properly closed

## Next Steps

1. Read [spec.md](./spec.md) for full requirements
2. Review [plan.md](./plan.md) for implementation phases
3. Follow [module-contract.md](./contracts/module-contract.md) for content structure
4. Start with Phase 0 tasks (repository setup)

## Support

- **Spec Questions**: Check `specs/004-physical-ai-book/spec.md`
- **Technical Issues**: Review research decisions in `research.md`
- **Content Standards**: Refer to Constitution at `.specify/memory/constitution.md`
