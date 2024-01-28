# FOC

## 1.将文件夹提交到一个新创建的github服务器

```bash
echo "# miniFOC" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/mailonghua/miniFOC.git
git push -u origin main
```

