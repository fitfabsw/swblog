---
title: "部落格使用指南"
date: 2026-03-05
author: Kevin Lee
description: "Hugo 部落格的完整使用指南：建立文章、Front Matter、Markdown 語法、自訂 Shortcodes"
tags: ["hugo", "markdown", "guide"]
math: true
ShowToc: true
TocOpen: true
---

本文整合了部落格所有寫作功能的說明，包含工作流程、文章建立、Front Matter 設定、Markdown 語法，以及自訂 Shortcodes 的完整用法。

<!--more-->

## 工作流程

本部落格使用 Hugo 靜態網站產生器，搭配 GitHub Actions + Self-hosted Runner 自動部署至公司內部 server。

### 整體流程

```
git pull                     # 1. 取得最新內容
hugo server -D               # 2. 本地預覽（含草稿）
# ... 編輯文章 ...
git add & git commit          # 3. 提交變更
git push                     # 4. 推送至 GitHub
                             # 5. GitHub Actions 自動建置並部署
```

### 詳細步驟

**首次使用 — Clone repo：**

```bash
git clone <repo-url>
cd swblog
```

**日常編輯：**

```bash
git pull                          # 取得最新內容
hugo server -D                    # 啟動本地預覽（http://localhost:1313）
```

編輯完成後推送：

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

部署完成後即可透過公司內部網路存取更新後的部落格：[https://10.35.40.81/swblog/](https://10.35.40.81/swblog/)

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

使用 Hugo CLI 建立文章：

```bash
hugo new posts/my-post.md           # 單檔模式
hugo new posts/my-post/index.md     # 資料夾模式
```

### 開發預覽

啟動本地開發伺服器預覽文章（`-D` 可顯示草稿）：

```bash
hugo server -D
```

預設網址為 `http://localhost:1313`。

---

## Front Matter 設定

Front Matter 是文章開頭以 `---` 包圍的 YAML 區塊，用來定義文章的元資料。以下列出所有可用欄位：

```yaml
---
title: "文章標題"              # 必填，文章標題
date: 2026-03-05               # 必填，發布日期
author: "Kevin Lee"            # 作者，也可用陣列：["Kevin Lee", "路人甲"]
description: "文章摘要"         # 搜尋引擎與社群分享用的描述
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
  alt: "替代文字"              # 圖片替代文字
  caption: "圖片說明"          # 圖片標題

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
##### H5
###### H6
```

### 段落

段落之間以空行分隔。在同一段落中換行需在行尾加兩個空格或使用 `<br>`。

### 粗體與斜體

```markdown
**粗體文字**
_斜體文字_
**_粗斜體_**
```

**粗體文字**、_斜體文字_、**_粗斜體_**

### 引言（Blockquote）

```markdown
> 這是一段引言。
> **可以**在引言中使用 _Markdown 語法_。

> Don't communicate by sharing memory, share memory by communicating.
>
> — Rob Pike
```

> Don't communicate by sharing memory, share memory by communicating.
>
> — Rob Pike

### 表格

```markdown
| Name  | Age |
| ----- | --- |
| Bob   | 27  |
| Alice | 23  |
```

| Name  | Age |
| ----- | --- |
| Bob   | 27  |
| Alice | 23  |

表格中可使用行內 Markdown：

| Italics   | Bold     | Code   |
| --------- | -------- | ------ |
| _italics_ | **bold** | `code` |

### 程式碼

**行內程式碼：**

```markdown
`inline code`
```

`inline code`

**程式碼區塊（指定語言與行號）：**

````markdown
```html {linenos=true}
<!DOCTYPE html>
<html lang="en">
    <head>
        <meta charset="utf-8" />
        <title>Example</title>
    </head>
    <body>
        <p>Hello</p>
    </body>
</html>
```
````

**Hugo highlight shortcode：**

```
{{</* highlight python */>}}
def hello():
    print("Hello, World!")
{{</* /highlight */>}}
```

{{< highlight python >}}
def hello():
    print("Hello, World!")
{{< /highlight >}}

### 清單

**有序清單：**

```markdown
1. First item
2. Second item
3. Third item
```

1. First item
2. Second item
3. Third item

**無序清單：**

```markdown
- List item
- Another item
- And another item
```

- List item
- Another item
- And another item

**巢狀清單：**

```markdown
- Fruit
    - Apple
    - Orange
- Dairy
    - Milk
    - Cheese
```

- Fruit
    - Apple
    - Orange
- Dairy
    - Milk
    - Cheese

**待辦清單：**

```markdown
- [x] 已完成項目
  - [x] 子項目
- [ ] 未完成項目
```

- [x] 已完成項目
  - [x] 子項目
- [ ] 未完成項目

### 定義列表

```markdown
Cat
: Fluffy animal everyone likes

Internet
: Vector of transmission for pictures of cats
```

Cat
: Fluffy animal everyone likes

Internet
: Vector of transmission for pictures of cats

### 腳註

```markdown
這是一段有腳註的文字。[^1]

[^1]: 這是腳註的內容。
```

這是一段有腳註的文字。[^1]

[^1]: 這是腳註的內容。

### HTML 元素

Markdown 中可直接使用 HTML：

```html
<abbr title="Graphics Interchange Format">GIF</abbr> 是一種圖片格式。

H<sub>2</sub>O

X<sup>n</sup> + Y<sup>n</sup> = Z<sup>n</sup>

按 <kbd><kbd>CTRL</kbd>+<kbd>ALT</kbd>+<kbd>Delete</kbd></kbd> 結束工作階段。

使用 <mark>螢光標記</mark> 強調文字。
```

<abbr title="Graphics Interchange Format">GIF</abbr> 是一種圖片格式。

H<sub>2</sub>O

X<sup>n</sup> + Y<sup>n</sup> = Z<sup>n</sup>

按 <kbd><kbd>CTRL</kbd>+<kbd>ALT</kbd>+<kbd>Delete</kbd></kbd> 結束工作階段。

使用 <mark>螢光標記</mark> 強調文字。

### Emoji

在 Hugo 設定中啟用 `enableEmoji: true` 後，可直接使用 emoji shortcode：

```markdown
:see_no_evil: :hear_no_evil: :speak_no_evil:
```

:see_no_evil: :hear_no_evil: :speak_no_evil:

完整列表參考 [Emoji cheat sheet](http://www.emoji-cheat-sheet.com/)。

### 數學公式（KaTeX）

在 Front Matter 中設定 `math: true` 即可啟用 KaTeX 數學公式渲染。

首先需在文章中載入 KaTeX（每篇文章只需一次）：

```
{{</* math.inline */>}}
{{ if or .Page.Params.math .Site.Params.math }}
<!-- KaTeX -->
<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/katex@0.11.1/dist/katex.min.css" ...>
<script defer src="https://cdn.jsdelivr.net/npm/katex@0.11.1/dist/katex.min.js" ...></script>
<script defer src="https://cdn.jsdelivr.net/npm/katex@0.11.1/dist/contrib/auto-render.min.js" ... onload="renderMathInElement(document.body);"></script>
{{ end }}
{{</* /math.inline */>}}
```

{{< math.inline >}}
{{ if or .Page.Params.math .Site.Params.math }}
<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/katex@0.11.1/dist/katex.min.css" integrity="sha384-zB1R0rpPzHqg7Kpt0Aljp8JPLqbXI3bhnPWROx27a9N0Ll6ZP/+DiW/UqRcLbRjq" crossorigin="anonymous">
<script defer src="https://cdn.jsdelivr.net/npm/katex@0.11.1/dist/katex.min.js" integrity="sha384-y23I5Q6l+B6vatafAwxRu/0oK/79VlbSz7Q9aiSZUvyWYIYsd+qj+o24G5ZU2zJz" crossorigin="anonymous"></script>
<script defer src="https://cdn.jsdelivr.net/npm/katex@0.11.1/dist/contrib/auto-render.min.js" integrity="sha384-kWPLUVMOks5AQFrykwIup5lo0m3iMkkHrD0uJ4H5cjeGihAutqP0yW0J6dpFiVkI" crossorigin="anonymous" onload="renderMathInElement(document.body);"></script>
{{ end }}
{{</ math.inline >}}

**行內公式：** 使用 `math.inline` shortcode 搭配 `\(` 和 `\)` 包圍：

```
{{</* math.inline */>}}
<p>行內公式：\(\varphi = \dfrac{1+\sqrt5}{2} = 1.6180339887…\)</p>
{{</* /math.inline */>}}
```

{{< math.inline >}}
<p>行內公式：\(\varphi = \dfrac{1+\sqrt5}{2}= 1.6180339887…\)</p>
{{</ math.inline >}}

**區塊公式：** 使用 `$$` 包圍：

```
$$
\varphi = 1+\frac{1} {1+\frac{1} {1+\frac{1} {1+\cdots} } }
$$
```

$$
 \varphi = 1+\frac{1} {1+\frac{1} {1+\frac{1} {1+\cdots} } }
$$

支援的函數參考 [KaTeX Supported Functions](https://katex.org/docs/supported.html)。

---

## 自訂 Shortcodes

本部落格提供以下自訂 shortcodes，使用語法為 `{{</* shortcode-name */>}}`。

### figure — 單張圖片

顯示單張圖片，支援標題與替代文字：

```
{{</* figure src="sample-1.jpg" alt="範例圖片" caption="這是圖片說明" */>}}
```

{{< figure src="sample-1.jpg" alt="範例圖片" caption="這是圖片說明" >}}

參數：
- `src`：圖片路徑（必填）
- `alt`：替代文字
- `caption`：圖片說明
- `title`：圖片標題
- `link`：點擊圖片的連結
- `width`：最大寬度

### figure-columns / figure-item — 並排圖片

將多張圖片以等寬網格並排顯示：

```
{{</* figure-columns */>}}
    {{</* figure-item "sample-2.jpg" */>}}
    {{</* figure-item "sample-3.jpg" */>}}
    {{</* figure-item "sample-4.jpg" */>}}
{{</* /figure-columns */>}}
```

{{< figure-columns >}}
    {{< figure-item "sample-2.jpg" >}}
    {{< figure-item "sample-3.jpg" >}}
    {{< figure-item "sample-4.jpg" >}}
{{< /figure-columns >}}

`figure-columns` 可傳入寬度參數，如 `{{</* figure-columns "80%" */>}}`。

`figure-item` 第一個參數為圖片路徑，第二個（選填）為圖片說明文字。

### gallery / load-photoswipe — 圖片畫廊

顯示資料夾中所有圖片，可放大滑動瀏覽：

```
{{</* gallery dir="gallery/sample-gallery" /*/>}}
{{</* load-photoswipe */>}}
```

{{< gallery dir="gallery/sample-gallery" />}}
{{< load-photoswipe >}}

注意事項：
- 圖片須放在 `/static/gallery/` 目錄中
- 一篇文章可以有多個 `gallery`，但只需一個 `load-photoswipe`
- `load-photoswipe` 須放在所有 `gallery` 之後

### youtube — YouTube 影片

嵌入 YouTube 影片：

```
{{</* youtube embed="VQraviuwbzU" */>}}
```

{{< youtube embed="VQraviuwbzU" >}}

搭配 CSS 可並排多個影片：

```html
<div style="display: flex">
  {{</* youtube embed="nJ81DFmgHdU" */>}}
  {{</* youtube embed="oMpqj_nMsg0" */>}}
</div>
```

<div style="display: flex">
  {{< youtube embed="nJ81DFmgHdU" >}}
  {{< youtube embed="oMpqj_nMsg0" >}}
</div>

### mxyoutube — YouTube 影片（無相關影片）

與 `youtube` 類似，但在暫停和結束時不顯示相關影片推薦：

```
{{</* mxyoutube embed="VIDEO_ID" */>}}
```

> **注意：** 此 shortcode 模板目前存在相容性問題（混用 positional 與 named params），建議優先使用 `youtube`。

### awesome — FontAwesome 圖示

顯示 [Font Awesome](https://fontawesome.com/icons) 圖示：

```
{{</* awesome fa-solid fa-cake-candles */>}}
{{</* awesome fa-brands fa-docker */>}}
{{</* awesome fa-solid fa-face-kiss-wink-heart */>}}
```

{{< awesome fa-solid fa-cake-candles >}} {{< awesome fa-brands fa-docker >}} {{< awesome fa-solid fa-face-kiss-wink-heart >}}

### details — 摺疊區塊

建立可展開/收合的摺疊區塊：

```
{{</* details "點擊展開" */>}}
這裡是被隱藏的內容。
支援 **Markdown** 語法。
{{</* /details */>}}
```

{{< details "點擊展開範例" >}}
這裡是被隱藏的內容。支援 **Markdown** 語法。
{{< /details >}}

### collapse — 摺疊區塊（進階）

與 `details` 類似，但使用具名參數，支援預設展開：

```
{{</* collapse summary="點擊展開 collapse" openByDefault=true */>}}
這是預設展開的摺疊內容。
{{</* /collapse */>}}
```

{{< collapse summary="點擊展開 collapse" openByDefault=true >}}
這是預設展開的摺疊內容。
{{< /collapse >}}

### blockquote — 引言區塊（附出處）

建立帶有作者、來源資訊的引言區塊：

```
{{</* blockquote author="Steve Jobs" source="Stanford Commencement Speech" link="https://news.stanford.edu/stories/2005/06/youve-got-find-love-jobs-says" title="Stanford News" */>}}
Stay hungry, stay foolish.
{{</* /blockquote */>}}
```

{{< blockquote author="Steve Jobs" source="Stanford Commencement Speech" link="https://news.stanford.edu/stories/2005/06/youve-got-find-love-jobs-says" title="Stanford News" >}}
Stay hungry, stay foolish.
{{< /blockquote >}}

參數：
- `author`：作者
- `source`：來源名稱
- `link`：來源連結
- `title`：連結顯示文字

### br — 換行

插入一個或多個空行：

```
段落一
{{</* br 3 */>}}
段落二
```

段落一
{{< br 3 >}}
段落二

數字參數表示 `<br>` 的數量，不帶參數則插入一個換行。

### table — 表格樣式

為 Markdown 表格添加自訂 CSS class：

```
{{</* table "striped" */>}}
| 語言     | 用途         |
| -------- | ------------ |
| Go       | 後端服務     |
| Python   | 資料分析     |
| JavaScript | 前端開發   |
{{</* /table */>}}
```

{{< table "striped" >}}
| 語言     | 用途         |
| -------- | ------------ |
| Go       | 後端服務     |
| Python   | 資料分析     |
| JavaScript | 前端開發   |
{{< /table >}}

### rawhtml — 原始 HTML

直接輸出原始 HTML，不經過 Markdown 處理：

```
{{</* rawhtml */>}}
<p style="color: darkorange; font-weight: bold;">這是透過 rawhtml 輸出的橘色粗體文字。</p>
{{</* /rawhtml */>}}
```

{{< rawhtml >}}
<p style="color: darkorange; font-weight: bold;">這是透過 rawhtml 輸出的橘色粗體文字。</p>
{{< /rawhtml >}}

### ltr / rtl — 文字方向

設定區塊的文字方向（左到右 / 右到左）：

```
{{</* rtl */>}}
هذا نص من اليمين إلى اليسار
{{</* /rtl */>}}
```

{{< rtl >}}
هذا نص من اليمين إلى اليسار
{{< /rtl >}}

```
{{</* ltr */>}}
This text flows left to right.
{{</* /ltr */>}}
```

{{< ltr >}}
This text flows left to right.
{{< /ltr >}}

傳入參數可啟用 Markdown 渲染：

```
{{</* rtl md=true */>}}
**粗體** 也可以使用
{{</* /rtl */>}}
```

{{< rtl md=true >}}
**粗體** 也可以使用
{{< /rtl >}}

---

## 在 Markdown 中使用自訂 CSS

可直接在文章中使用 HTML `style` 屬性：

```html
<div style="font-size: 0.8em; font-style: italic">
自訂樣式的文字內容
</div>
```

<div style="font-size: 0.8em; font-style: italic">
自訂樣式的文字內容
</div>
