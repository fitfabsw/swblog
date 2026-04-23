# CPD 軟體部落

Hugo 靜態網站，使用 [PaperMod](https://github.com/adityatelange/hugo-PaperMod) 主題，搭配 GitHub Actions + Self-hosted Runner 自動部署至公司內部 server。

---

## 工作流程

### 整體流程

```bash
git pull                     # 1. 取得最新內容
hugo server -D               # 2. 本地預覽（含草稿）
# ... 編輯文章 ...
git add & git commit         # 3. 提交變更
git push                     # 4. 推送至 GitHub
                             # 5. GitHub Actions 自動建置並部署
```

### 環境準備

本部落格需要 **Hugo Extended** 和 **Go**。

**安裝 Hugo：**

```bash
# macOS
brew install hugo

# Ubuntu / Debian
sudo apt install hugo

# Windows (Scoop)
scoop install hugo
```

**安裝 Go（Hugo Modules 需要）：**

```bash
# macOS
brew install go

# Ubuntu / Debian
sudo apt install golang
```

**驗證安裝：**

```bash
hugo version    # 確認顯示 extended 版本
go version
```

### 詳細步驟

**首次使用 — Clone repo：**

```bash
git clone https://github.com/fitfabsw/swblog.git
cd swblog
```

**日常編輯：**

```bash
git pull                          # 取得最新內容
hugo server -D                    # 啟動本地預覽（http://localhost:1313）
```

**編輯完成後推送：**

```bash
git add content/posts/my-post/    # 暫存變更的檔案
git commit -m "Add new post"      # 提交
git push                          # 推送至 GitHub
```

### 自動部署

Push 到 `fit-cpd-sw` 分支後，GitHub Actions 會自動觸發部署：

1. Self-hosted Runner（公司內部 server）接收任務
2. 執行 `hugo --minify` 建置靜態網站
3. 將產出的檔案部署至 `/var/www/swblog/`

部署完成後可透過公司內部網路存取：https://10.35.40.81/swblog/

---

## 如何建立一篇文章

### 檔案結構

Hugo 文章有兩種組織方式：

**單檔模式** — 適合純文字文章，不含本地圖片：

```
content/posts/my-post.md
```

**資料夾模式** — 適合包含圖片或附件的文章：

```
content/posts/my-post/
├── index.md
├── image1.jpg
└── image2.png
```

資料夾模式中，圖片可直接以相對路徑引用（如 `![alt](image1.jpg)`）。

### 建立新文章

```bash
hugo new posts/my-post.md           # 單檔模式
hugo new posts/my-post/index.md     # 資料夾模式
```

### 開發預覽

```bash
hugo server -D    # -D 可顯示草稿，預設網址為 http://localhost:1313
```

---

## Front Matter 設定

Front Matter 是文章開頭以 `---` 包圍的 YAML 區塊，用來定義文章的元資料。

```yaml
---
title: "文章標題"              # 必填，文章標題
date: 2026-03-05               # 必填，發布日期
author: "Kevin Lee"            # 作者，也可用陣列：["Kevin Lee", "路人甲"]
description: "文章摘要"        # 搜尋引擎與社群分享用的描述
tags: ["hugo", "markdown"]     # 標籤
categories: ["教學"]           # 分類
series: ["Hugo 指南"]          # 系列文章
aliases: ["/old-url"]          # URL 別名，用於舊文章重導向
summary: "這是摘要"            # 文章列表中顯示的摘要
draft: true                    # 草稿，設為 true 則不會在正式建置中顯示

# 目錄
ShowToc: true                  # 顯示目錄
TocOpen: true                  # 預設展開目錄

# 封面圖片
cover:
  image: "cover.jpg"           # 圖片路徑（本地或 URL）
  alt: "替代文字"
  caption: "圖片說明"

# 顯示控制
ShowBreadCrumbs: true          # 顯示麵包屑導覽
hideMeta: true                 # 隱藏文章元資料（日期、作者等）
searchHidden: true             # 從搜尋中隱藏

# 數學公式
math: true                     # 啟用 KaTeX 數學公式渲染
---
```

---

## Markdown 語法

### 標題

```markdown
# H1
## H2
### H3
#### H4
```

### 粗體與斜體

```markdown
**粗體文字**
_斜體文字_
**_粗斜體_**
```

### 引言（Blockquote）

```markdown
> 這是一段引言。
> **可以**在引言中使用 _Markdown 語法_。
```

### 表格

```markdown
| Name  | Age |
| ----- | --- |
| Bob   | 27  |
| Alice | 23  |
```

### 程式碼

行內程式碼：`` `inline code` ``

程式碼區塊（指定語言與行號）：

````markdown
```html {linenos=true}
<!DOCTYPE html>
<html lang="en">
  <body><p>Hello</p></body>
</html>
```
````

Hugo highlight shortcode：

```
{{< highlight python >}}
def hello():
    print("Hello, World!")
{{< /highlight >}}
```

### 清單

```markdown
1. First item       # 有序
- List item         # 無序
- [x] 已完成        # 待辦清單
- [ ] 未完成
```

### 腳註

```markdown
這是一段有腳註的文字。[^1]

[^1]: 這是腳註的內容。
```

### Emoji

在 `hugo.yaml` 中啟用 `enableEmoji: true` 後，可直接使用 emoji shortcode：

```
:see_no_evil: :hear_no_evil: :speak_no_evil:
```

### 數學公式（KaTeX）

在 Front Matter 中設定 `math: true`，然後在文章中載入 KaTeX（每篇文章只需一次）：

```
{{< math.inline >}}
{{ if or .Page.Params.math .Site.Params.math }}
<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/katex@0.11.1/dist/katex.min.css">
<script defer src="https://cdn.jsdelivr.net/npm/katex@0.11.1/dist/katex.min.js"></script>
<script defer src="https://cdn.jsdelivr.net/npm/katex@0.11.1/dist/contrib/auto-render.min.js"
  onload="renderMathInElement(document.body);"></script>
{{ end }}
{{< /math.inline >}}
```

行內公式（使用 `\(` 和 `\)` 包圍）：

```
{{< math.inline >}}
<p>\(\varphi = \dfrac{1+\sqrt5}{2}\)</p>
{{< /math.inline >}}
```

區塊公式（使用 `$$` 包圍）：

```
$$
\varphi = 1+\frac{1}{1+\frac{1}{1+\cdots}}
$$
```

---

## 自訂 Shortcodes

### `figure` — 單張圖片

```
{{< figure src="sample.jpg" alt="範例圖片" caption="這是圖片說明" >}}
```

| 參數 | 說明 |
| --- | --- |
| `src` | 圖片路徑（必填） |
| `alt` | 替代文字 |
| `caption` | 圖片說明 |
| `title` | 圖片標題 |
| `link` | 點擊圖片的連結 |
| `width` | 最大寬度 |

### `figure-columns` / `figure-item` — 並排圖片

```
{{< figure-columns >}}
    {{< figure-item "sample-1.jpg" >}}
    {{< figure-item "sample-2.jpg" "圖片說明" >}}
{{< /figure-columns >}}
```

`figure-columns` 可傳入寬度參數，如 `{{< figure-columns "80%" >}}`。

### `gallery` / `load-photoswipe` — 圖片畫廊

```
{{< gallery dir="gallery/sample-gallery" />}}
{{< load-photoswipe >}}
```

- 圖片須放在 `/static/gallery/` 目錄中
- 一篇文章可以有多個 `gallery`，但只需一個 `load-photoswipe`
- `load-photoswipe` 須放在所有 `gallery` 之後

### `youtube` — YouTube 影片

```
{{< youtube embed="VQraviuwbzU" >}}
```

並排多個影片：

```html
<div style="display: flex">
  {{< youtube embed="nJ81DFmgHdU" >}}
  {{< youtube embed="oMpqj_nMsg0" >}}
</div>
```

### `mxyoutube` — YouTube 影片（無相關影片）

與 `youtube` 類似，但暫停和結束時不顯示相關影片推薦：

```
{{< mxyoutube embed="VIDEO_ID" >}}
```

> 注意：此 shortcode 目前存在相容性問題，建議優先使用 `youtube`。

### `awesome` — FontAwesome 圖示

```
{{< awesome fa-solid fa-cake-candles >}}
{{< awesome fa-brands fa-docker >}}
```

### `details` — 摺疊區塊

```
{{< details "點擊展開" >}}
這裡是被隱藏的內容。支援 **Markdown** 語法。
{{< /details >}}
```

### `collapse` — 摺疊區塊（進階）

```
{{< collapse summary="點擊展開" openByDefault=true >}}
這是預設展開的摺疊內容。
{{< /collapse >}}
```

### `blockquote` — 引言區塊（附出處）

```
{{< blockquote author="Steve Jobs" source="Stanford Commencement Speech"
    link="https://news.stanford.edu/..." title="Stanford News" >}}
Stay hungry, stay foolish.
{{< /blockquote >}}
```

### `br` — 換行

```
段落一
{{< br 3 >}}
段落二
```

數字參數表示 `<br>` 的數量，不帶參數則插入一個換行。

### `table` — 表格樣式

```
{{< table "striped" >}}
| 語言 | 用途 |
| ---- | ---- |
| Go   | 後端服務 |
{{< /table >}}
```

### `rawhtml` — 原始 HTML

```
{{< rawhtml >}}
<p style="color: darkorange; font-weight: bold;">橘色粗體文字</p>
{{< /rawhtml >}}
```

### `ltr` / `rtl` — 文字方向

```
{{< rtl >}}
هذا نص من اليمين إلى اليسار
{{< /rtl >}}
```

傳入 `md=true` 可啟用 Markdown 渲染：

```
{{< rtl md=true >}}
**粗體** 也可以使用
{{< /rtl >}}
```

### 在 Markdown 中使用自訂 CSS

可直接在文章中使用 HTML `style` 屬性：

```html
<div style="font-size: 0.8em; font-style: italic">
自訂樣式的文字內容
</div>
```
