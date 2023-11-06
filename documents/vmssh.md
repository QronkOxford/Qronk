# SSH into your VM

Instructions to access your virtual machine from Visual Studio Code on your host machine. Thanks to Shuyu Lin for help! 

Please contact [Jin Rhee](mailto:jin.rhee@sjc.ox.ac.uk) if you bump into major issues.

## Generating public key using ssh-keygen

You will now create a ssh-key, which uniquely identifies your host machine. To create a ssh-key, run the following:
```bash
% ssh-keygen -b 4096 -t rsa
Generating public/private rsa key pair.
Enter file in which to save the key (/Users/[user]/.ssh/id_rsa):
```
Unless you have a pre-existing public ssh-key, simply press enter when asked to name the file to save the key. This will result in ~/.ssh/ as id_rsa and id_rsa.pub.
Keep the passphrase blank by pressing enter:
```bash
Enter passphrase (empty for no passphrase): 
Enter same passphrase again:
...
```

If done correctly, a key fingerprint and its randomart will be shown. To print your newly created key, run the following:

```bash
% cat ~/.ssh/id_rsa.pub
[Your SSH key]
```

Copy your ssh key (which starts with 'ssh-rsa' and ends with 'user@MacBook-Pro.local' or your equivalent host machine).

## Adding your public key to your virtual machine

Your virtual machine needs to have your public key to identify you. We will now save it in your VM environment. Access the shell prompt open the following file on your favourite text editor:
```bash
$ nano ~/.ssh/authorized_keys
```
Paste your key generated from above into the file. Save and close (you can check if your key has been added by running `cat ~/.ssh/authorized_keys`).

## Changing your user password

Without change, your virtual machine will ask for your user's password when you try to remotely access it through ssh (a reasonable request). If you have newly created your virtual machine instance, your user will likely be `ubuntu`, and you will not know what this user's password is. Change your user's password by running:
```bash
$ sudo passwd ubuntu
New password: 
Retype new password: 
```
Enter your password into the prompt, and retype your password.

## Using VSCode to remotely access your virtual machine

To access your virtual machine from VSCode, you will need to download the Remote SSH Extension. Follow 'Install the extension' and 'Connect using SSH' instructions in this [link](https://code.visualstudio.com/docs/remote/ssh-tutorial#_install-the-extension)

You can get your virtual machine instance's ip address by using the following:
```bash
% multipass list
...
preferable-duckling     Running           192.168.64.8     Ubuntu 22.04 LTS
...
```


