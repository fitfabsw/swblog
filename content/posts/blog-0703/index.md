---
title: "部落格文章"
summary: 這是此篇文章的總結
draft: false
date: 2022-07-03
ShowToc: true
TocOpen: true
aliases: ["/this-is-an-aliase"]
tags: ["旅遊"]
author: ["Kevin Lee", "路人甲"]
cover:
  image: gallery/posts-2021-07-03-A/IMG_7603.jpg
  alt: dummy alt
  caption: This is a caption
ShowBreadCrumbs: true
---

## Markdown
### Table
#### Example1
Name    | Age
--------|------
Bob     | 27
Alice   | 23

#### Example2
| Misclassified Image | Dog | Zebra | Blurry |
| ------------------- | --- | ----- | ------ |
| 1                   | ✓   |       |        |
| 2                   |     | ✓     |        |
| 3                   |     | ✓     | ✓      |
| N                   | ... |       |        |
| Total               | 8%  | 43%   | 61%    |

### Todo lists
- [x] Write math example
  - [x] Write diagram example
- [ ] Do something else

### Lists
Cat
: Fluffy animal everyone likes

Internet
: Vector of transmission for pictures of cats

### footnote
This is a footnote.[^1]

[^1]: the footnote text.


## Shortcodes
### 圖片
> 使用shortcode `figure`顯示單張照片
```
{{</* figure src="IMG_7000.jpg" */>}}
```
{{< figure src="IMG_7000.jpg" alt="SDF">}}

### 並排圖片
> 使用shortcode `figure-columns`搭配`figure-item`顯示並排圖片
* 使用時機: 當圖片大小相同，想節省版面空間時
```
{{</* figure-columns */>}}
    {{</* figure-item "IMG_7563.jpg" */>}}
    {{</* figure-item "IMG_7680.jpg" */>}}
    {{</* figure-item "IMG_8634.jpg" */>}}
{{</* /figure-columns */>}}
```
{{< figure-columns >}}
    {{< figure-item "IMG_7563.jpg" >}}
    {{< figure-item "IMG_7680.jpg" >}}
    {{< figure-item "IMG_8634.jpg" >}}
{{< /figure-columns >}}

### 圖庫
> 使用shortcode `gallery`搭配`load-photoswipe`顯示照片群組，可放大滑動瀏覽。
* 注意圖片須放在/static/gallery/中。
* 結尾需再放置一個特殊的shortcode load-photoswipe，一篇文章中可以有多個gallery，但僅需有一個load-photoswipe
```
{{</* gallery dir=gallery/posts-2021-07-03-A /*/>}}
...
{{</* load-photoswipe /*/>}}
```
{{< gallery dir="gallery/posts-2021-07-03-A" />}}
{{< load-photoswipe >}}

### 圖示
> 使用shortcode `awesome`顯示awesome font

* {{< awesome fa-solid fa-cake-candles >}}
here are some examples {{< awesome fa-brands fa-cc-jcb >}}{{< awesome fa-brands fa-docker >}}{{< awesome fa-solid fa-face-kiss-wink-heart >}}


### 折疊
> 使用shortcode `details`折疊區塊

{{< details  這是一段可以摺疊的區塊 >}}
**Collapsed text**

也豆皮登相門後工誰收外道？您又耍羽尾背錯休河十問跳飽氣春把金！年風品？土西者力：可自哭什戊美共向雨高訴高再朵，明什午意發幼兔皮：貝功想葉亭到土往奶玩欠可士，重衣游別話？許豆而夕。
苦風金斥。尤位我多由立空里己告肖飽蝴哪月帽、包干丁旦細活，浪她年或視包吧土主學耳過坡呢。美肖旁信三荷。

**Collapsed gallery**
{{</* gallery dir=gallery/posts-2021-07-03-B /*/>}}
{{< /details >}}

### Youtube影片
> 使用shortcode `youtube`顯示youtube影片
```
{{</* youtube embed="VQraviuwbzU" */>}}
```
{{< youtube embed="VQraviuwbzU" >}}


### 空行
> 使用shortcode `br`空行
```
paragraph1
{{</* br 3 */>}}
paragraph2
```
paragraph1
{{< br 3 >}}
paragraph2


## 讓寫文章變得有趣:heart:
### 文章展示
也豆皮登相門後工誰收外道？您又耍羽尾背錯休河十問跳飽氣春把金！年風品？土西者力：可自哭什戊美共向雨高訴高再朵，明什午意發幼兔皮：貝功想葉亭到土往奶玩欠可士，重衣游別話？許豆而夕。
苦風金斥。尤位我多由立空里己告肖飽蝴哪月帽、包干丁旦細活，浪她年或視包吧土主學耳過坡呢。美肖旁信三荷。

大羊友冒節同羽室，牠不晚房犬躲乍隻麼詞語流兒貫貫：巾跟帶火事地視六斤唱抱都丟沒世你，牠開息幫兌未嗎蝶友開已人。西鴨課把。
> 結論: 寫好文做好事

### 在markdown中套用自訂css
```
<div style="font-size: 0.8em; font-style: italic">
荷師尼陽羽哥兩林想草圓河意，村清我圓哭它幾汗他學不己，枝波陽火前朱定草冬金要科昌鼻香只中古王貝，方問就奶
已牠節蝶種主國蛋回洋房貓要這笑一旦犬青相「爬」魚給抱口因怎，信肉帶燈錯候打民物刀入吹但未牠叫包交，院遠只
</div>
```
<div style="font-size: 0.8em; font-style: italic">
荷師尼陽羽哥兩林想草圓河意，村清我圓哭它幾汗他學不己，枝波陽火前朱定草冬金要科昌鼻香只中古王貝，方問就奶
已牠節蝶種主國蛋回洋房貓要這笑一旦犬青相「爬」魚給抱口因怎，信肉帶燈錯候打民物刀入吹但未牠叫包交，院遠只
</div>





* 搭配css製作並排youtube影片

```
<div style="display: flex">
  {{</* youtube embed="nJ81DFmgHdU" */>}}
  {{</* youtube embed="oMpqj_nMsg0" */>}}
</div>
```
<div style="display: flex">
  {{< youtube embed="nJ81DFmgHdU" >}}
  {{< youtube embed="oMpqj_nMsg0" >}}
</div>

