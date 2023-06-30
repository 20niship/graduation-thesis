# Makefile

# 変数の定義
TEX_COMPILER = pdflatex
OUTPUT_DIR = build

# ディレクトリ内のすべてのtexファイルを取得
TEX_FILES := $(wildcard *.tex)

# デフォルトのターゲット
all: pdf

# テキストファイルをPDFに変換するターゲット
pdf: $(TEX_FILES:%.tex=$(OUTPUT_DIR)/%.pdf)

# 各ファイルの依存関係と変換ルールを指定
$(OUTPUT_DIR)/%.pdf: %.tex | $(OUTPUT_DIR)
	$(TEX_COMPILER) -interaction=nonstopmode -output-directory=$(OUTPUT_DIR) -file-line-error $< > /dev/null

# 出力ディレクトリが存在しない場合は作成
$(OUTPUT_DIR):
	mkdir -p $(OUTPUT_DIR)

# クリーンアップターゲット
clean:
	rm -rf $(OUTPUT_DIR)
