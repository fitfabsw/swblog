## 專案結構

```
arch-asyncio-dag/
├── core/                         # 核心執行引擎
│   ├── async_dag_executor.py     # DAG 執行器（async + DAG）
│   ├── async_flow_controller.py  # 流程控制器，協調各執行器
│   ├── resource_manager.py       # 資源設定與查詢
│   └── serial_manager.py         # 序列埠管理
├── actions/                      # 動作系統（步驟實作）
│   ├── serial.py                 # 序列通訊動作（主要為 send/wait）
│   └── common.py                 # 通用／工具動作
├── models/                       # 資料模型與列舉
│   └── flow_models.py            # 流程／步驟模型 + Status/Result 列舉
├── adapters/                     # 介面層
│   └── qt_flow_adapter.py        # Qt 介面：控制器 → 樹狀資料與訊號
├── views/                        # UI 視圖層
│   ├── qml/                      # QML 介面（MainWindow、MainPanel、TestTableView）
│   │   └── components/           # 可重用 QML 元件
│   └── qml_bridge.py             # FlowTestBridge：將後端暴露給 QML
├── applications/                 # 應用程式進入點
│   ├── cli/                      # 命令列執行
│   └── gui/                      # 以 QML 為基礎的 GUI 應用
├── flows/                        # 流程定義
│   ├── Main*.csv                 # 主流程檔
│   └── tech/*.csv                # 技術流程檔
├── utils/                        # 工具函式
│   └── serial_utils.py           # 序列埠偵測輔助
├── config.yaml                   # 資源設定（序列埠等）
└── README.md
```

## 主要功能

### **Asyncio 優先設計**

- **原生 async/await**：可同時處理大量任務，提高資源效率，並讓 UI 更新保持即時不延遲。
- **非阻塞操作**：所有 I/O 皆為非同步

### **完整執行引擎**

- **依賴解析**：自動解析與管理複雜步驟依賴
- **雙層結構**：Main CSV 定義測試項目，Tech CSV 定義細部步驟
- **進階功能**：狀態追蹤、日誌與結果管理

### 資源管理

- **設定驅動**：透過 `config.yaml` 集中管理資源
- **序列裝置支援**：統一的序列埠通訊介面
- **資源衝突偵測**：執行時自動偵測並避免資源衝突

### **GUI**

- **Qt Quick + QML**：以 Qt Quick 建構的現代介面（`MainWindow.qml`、`MainPanel.qml`、`TestTableView.qml`）
- **Python 橋接**：`FlowTestBridge`（`views/qml_bridge.py`）將流程、狀態與樹狀模型暴露給 QML
- **即時監控**：依序列埠分表的即時執行狀態更新
- **回應式設計**：非同步執行時 UI 不阻塞

### **簡易動作系統**

- **直接函式對應**：`serial.send` 直接對應到 `serial.py` 的 `send()` 函式
- **直覺 API**：比類別式寫法更易讀、易改
- **動態發現**：自動註冊與發現動作

## 安裝與設定

### 複製專案至本地端

```bash
git clone https://github.com/fitfabsw/facty_python.git
cd facty_python
git checkout arch-asyncio-dag-qml
```

### 使用 uv 創建虛擬環境

[uv](https://docs.astral.sh/uv/) 是一款快速的 Python 套件安裝及解析工具。如果尚未安裝 uv，請先安裝：

```bash
# macOS/Linux
curl -LsSf https://astral.sh/uv/install.sh | sh

# 或使用 pip
pip install uv
```

建立虛擬環境並安裝相依套件：

```bash
uv sync
```

這會在專案根目錄建立 `.venv`，並依 `pyproject.toml` 安裝所有相依套件。


## 快速開始

### GUI 應用程式

```bash
uv run arch-asyncio-dag/applications/gui/app.py arch-asyncio-dag/flows/Main.csv
```

### CLI 測試

```bash
uv run arch-asyncio-dag/applications/cli/async_single_tester.py arch-asyncio-dag/flows/Main.csv
```

## 設定

編輯 `config.yaml` 以設定資源：

```yaml
resources:
  - name: "serial0"
    port: "/dev/cu.usbmodem*"
    baudrate: 115200
```

### 資源設定說明

- **序列資源**：以存在 `port` 欄位識別
- **資源命名**：每個資源須有唯一的 `name` 欄位
- **獨佔存取**：步驟執行期間獨佔鎖定資源，避免衝突
- **設定方式**：資源專用設定（如序列埠的 `baudrate`）寫在該資源條目的頂層

## 流程 CSV 欄位參考

流程採雙層 CSV：**Main CSV** 定義測試項目與對應的 tech 檔；**Tech CSV** 定義步驟及其欄位（如下表）。

### Main CSV（例如 `MainSimple.csv`）

| 欄位         | 說明 |
|-------------|------|
| **test_item** | 測試項目（群組）名稱，須唯一。 |
| **tech**      | Tech 流程檔名，位於 `flows/tech/`，不含 `.csv`。例：`dut` → `flows/tech/dut.csv`。 |

### Tech CSV（例如 `flows/tech/dut.csv`）

| 欄位             | 必填 | 說明 |
|------------------|------|------|
| **test_item**    | 首列 | 測試項目名稱，須與 Main CSV 一致；後續列可留空（同群組）。 |
| **id**           | 是   | 此測試項目內的步驟 ID，供 `after` 依賴參考。 |
| **description**  | 否   | 人類可讀描述。 |
| **action**       | 是   | 動作名稱（如 `serial.send`、`common.delay`），對應 `actions/` 中的函式。 |
| **command**      | 視動作 | 命令字串（如序列命令）。換行請用 `\n`。 |
| **after**        | 否   | 此步驟所依賴的步驟 ID，逗號分隔。僅在所列步驟完成後執行。 |
| **timeout**      | 否   | 等待**預期回應**（如 `wait_for.string`）的秒數。預設 `1.0`。用於 `serial.send`。 |
| **needs_resources** | 否 | 資源名稱，逗號分隔。步驟執行前會獨佔取得這些資源。 |
| **delay**        | 否   | 在**執行此步驟前**延遲秒數（依賴滿足後）。預設 `0.0`。 |
| **exit_early**   | 否   | 若為 `true`，此步驟失敗會中止整個流程。預設 `false`。 |
| **retry**        | 否   | 保留供日後使用。 |
| **parameters**   | 否   | 動作專用選項的 JSON。例：`{"wait_for": {"string": "> ft:ok"}}`、`{"seconds": 2}`。 |
