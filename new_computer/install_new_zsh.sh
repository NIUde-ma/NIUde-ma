#!/bin/bash
sudo apt-get update
sudo apt-get install zsh
chsh -s /bin/zsh
sudo vim /etc/passwd #把第一行的/bin/bash改成/bin/zsh，这个是root用户的。
sudo apt-get install git
sh -c "$(curl -fsSL https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh)"
sudo apt-get install autojump
vim .zshrc
#在最后一行加入，注意点后面是一个空格
. /usr/share/autojump/autojump.sh
source ~/.zshrc
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git
echo "source ${(q-)PWD}/zsh-syntax-highlighting/zsh-syntax-highlighting.zsh" >> ${ZDOTDIR:-$HOME}/.zshrc
source ~/.zshrc
git clone https://gitee.com/hailin_cool/zsh-autosuggestions.git $ZSH_CUSTOM/plugins/zsh-autosuggestions
plugins=(zsh-autosuggestions)
vim ~/.zshrc #将plugins（git）中加入zsh-autosuggestions
source $ZSH_CUSTOM/plugins/zsh-autosuggestions/zsh-autosuggestions.zsh
source ~/.zshrc



