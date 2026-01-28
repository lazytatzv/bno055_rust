## fish setup

in ~/.config/fish/config.fish

```fish
# direnvをfishで有効化
direnv hook fish | source
```

In this directory

```bash
echo "use flake" > .envrc
direnv allow
```
