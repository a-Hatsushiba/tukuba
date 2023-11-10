# waypoint_tool.cppに関するREADMEファイル

## 注意点
  - 依存ライブラリとして rs_navi, rs_common を`make install`しておく.
  - build内でMakeする.
## 操作マニュアル
1. 起動方法
  - コマンドライン上で以下の通り実行.
  `./waypoint_tool [マップ画像] [読み込みwaypointファイル(.csv)] [保存waypointファイル(.csv)]`

  - この時、読み込みwaypointファイルがない場合は保存waypointファイルと同様のもので良い.

2. 基本操作
- マウス操作
  - INSERT_*モード以外はwaypointがある場合は最も近い点を赤点で示す.
  - INSERT_*モードではマウス位置(waypoint入力位置)を赤点で示す.
  - DELETE_MULTI, EDIT_MULTIモードでは最初の選択位置を紫の点で示す.
  - EDIT_*モードでは選択したwaypointを紫の点で示す.
- キー操作
  - 通常時
    - マップ移動 : w, a, s, d
    - EDITモード変更 : e 
      (VIEW, INSERT_SINGLE, INSERT_MULTI, DELETE_SINGLE, DELETE_MULTI, EDIT_SINGLE, EDIT_MULTI)
    -  マップの拡大: ;
    -  マップの縮小: -
    - waypoint保存 : S(shift+s)
    - 変更等せずに終了 : qキー
  -  EDITモードがEDIT_* (通常時にプラスして)
     - waypointの移動 : w, a, s, d (waypointが選択されている場合)
     - Velocityの値を上げる : i
     - Velocityの値を下げる : u
     - Lane.rightの値を上げる : l (Lの小文字)
     - Lane.rightの値を下げる : k
     - Lane.leftの値を上げる : j
     - Lane.leftの値を下げる : h
     - Avoid modeの変更 : m
     - Navigation modeの変更 : n
## 関数説明(メンテナンス用)
- 後で更新