---
title: "SW_TestStation_Introduction"
date: 2026-03-16T10:35:31+08:00
author: "Alan Hung"
description: "Test_Station測站介紹"         # 搜尋引擎與社群分享用的描述
tags: ["Test_Station"]     # 標籤
categories: ["教學"]           # 分類
series: ["Test_Station 指南"]          # 系列文章
summary: "Test_Station測站介紹"            # 文章列表中顯示的摘要
draft: False                    # 草稿，設為 true 則不會在正式建置中顯示

# 目錄
ShowToc: true                  # 顯示目錄
TocOpen: true                  # 預設展開目錄

# 封面圖片
cover:
  image: "/img/brain_loading.jpeg" # 圖片路徑（本地或 URL）
  alt: "Atlas2"              # 圖片替代文字 


---



### 1.Test_Station測站介紹
<iframe src="/slides/SW_test_station_introducing_20260316_All-TestItem.pdf" width="100%" height="500px"></iframe>

<a href="/slides/SW_test_station_introducing_20260316_All-TestItem.pptx" download>
  <button style="
    padding:12px 20px;
    font-size:16px;
    background-color:#007bff;
    color:white;
    border:none;
    border-radius:8px;
    cursor:pointer;">
    📥 下載.pptx簡報
  </button>
</a>


</br> </br>


### 2.PPT 直接放進網站就能滑的方法
1. **把 PPT 轉成 PDF** - ex.slides.pdf
2. **放進 Hugo 的 static 資料夾** - 在 Hugo專案資料夾/static/建立 -> slides資料夾，把 slides.pdf 放進去
 - static/ 是一個專門放「不需要處理」的檔案資料夾，放入裡面的檔案會直接變成網站根目錄資源，適合任何要「直接被瀏覽器讀取」的檔案
3. **建立簡單滑動頁面** - 在content/建立.md檔案 內容加入
```html {linenos=true}
<iframe src="/slides/slides.pdf" width="100%" height="500px"></iframe>
```