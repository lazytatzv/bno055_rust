{
  # 外部から持ってくるもの
  inputs = {
    # urlでどこからダウンロードするか指定する
    # 数十万のパッケージが入った巨大なリポジトリ
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    # Rust用
    rust-overlay.url = "github:oxalica/rust-overlay";
  };

  # inputsを引数として受け取って、完成品を返す
  outputs = { self, nixpkgs, rust-overlay }:
    # 変数定義
    let 
      system = "x86_64-linux";

      pkgs = import nixpkgs {
        inherit system;
        overlays = [ (import rust-overlay) ];
      };

      # Rust toolchainのカスタマイズ
      myRust = pkgs.rust-bin.stable.latest.minimal.override {
        extensions = [
          "rust-src"
          "rust-analyzer"
          "clippy"
          "rustfmt"
        ];
        targets = [ "thumbv7em-none-eabihf" ]; # stm32のクロスコンパイル用
      };

    # 成果物そのもの
    in
    {
      devShells.${system}.default = pkgs.mkShell {
        buildInputs = [
          myRust
          pkgs.probe-rs-tools
          pkgs.flip-link
        ];
      }; 
    };

}
