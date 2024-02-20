# Git Cheatsheet

A high-level overview of how we'll use GitHub this semester.

---
## Installing GitHub Command Line Tools
You'll need to do a little installation/configuration to be able to interact with GitHub via your command line.  Don't worry, it's relatively painless.
- The instructions below come from https://github.com/cli/cli/blob/trunk/docs/install_linux.md, https://github.com/cli/cli#installation, and https://docs.github.com/en/github-cli/github-cli/quickstart


1.  Install GitHub CLI:
    ```
    type -p curl >/dev/null || (sudo apt update && sudo apt install curl -y)
    curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg \
    && sudo chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null \
    && sudo apt update \
    && sudo apt install gh -y
    ```

2. Authenticate with GitHub:
    ```
    gh auth login
    ```

3.  Follow the on-screen prompts.
    > GitHub CLI automatically stores your Git credentials for you when you choose HTTPS as your preferred protocol for Git operations and answer "yes" to the prompt asking if you would like to authenticate to Git with your GitHub credentials. This can be useful as it allows you to use Git commands like `git push` and `git pull` without needing to set up a separate credential manager or use SSH.


---

## 1) Initialize a Local Instance of the Repository on your Computer

*You'll perform this step **one time**, when you are first connecting your local computer to the GitHub server.*

1. Go to your `Projects` directory (I suggest that you put all of your projects within this directory) and create a directory for the course:
   ```
   cd ~/Projects
   mkdir IE-482-582
   cd IE-482-582
   ```
   
2. Clone from the online git repo into your local `Projects/IE-482-582` directory.  For the Spring 2024 semester, this will be:
   ```
   git clone https://github.com/IE-482-582/spring2024.git
   ```
   This will create a directory named `spring2024`.
   
   **Note:** You'll get an error if you try to clone a repo that is already on your computer.  If this happens, you'll need to delete the existing repo from your computer first.  
   - **CAUTION** The following command cannot be undone:
       ```
       # ONLY DO THIS IF YOU GET THE ERROR ABOVE
       rm -rf spring2024
       # AFTER REMOVING (DELETING) spring2024 TRY TO `git clone` AGAIN
       ```  
      
3. CD into the `spring2024` directory
   ```
   cd spring2024
   ```
   
4. Check out your branch (or create a new branch).  `BRANCH` should be your GitHub username.
   ```
   git checkout -b BRANCH
   ```
 
## 2) Keep your Local Branch Up-to-date with the Master/Main Branch
*You don't want your branch to fall behind the main branch.*

**You should perform these steps BEFORE you start making changes to your local branch.**

1. Create a backup of your local `spring2024` directory, just in case your local code gets overwritten by the command in Step 2.  You'll need to be "up" one level in the directory tree (so you can backup the entire directory of your repo).
	```
	cd ~/Projects/IE-482-582
	tar -chjvf spring2024_backup_$(date "+%Y%m%dT%H%M%S").tar.bz2 spring2024
	ls      # Make sure the .tar.bz2 file was created
	```
   - The `tar` command will create a backup archive with the current timestamp.

2. The next command will sync your local files (in your branch) with the main branch   
   ```
   cd ~/Projects/IE-482-582/spring2024
   git pull origin main
   ```

At this point your branch should include the latest code from the `main` branch, plus any new changes that you've made.


## 3) Push to your Branch
*This will ensure that your local code is uploaded to your branch on the Github server.*

**You should follow these steps AFTER you make changes to code on your computer.  PLEASE PUSH TO YOUR BRANCH AT THE END OF EVERY DAY.**
    	
1. Check if there are differences between your **local** branch and your **remote** (online) branch:
   ```
   cd ~/Projects/IE-482-582/spring2024
   git status
   ```
   	
2. Add changes from your local directory.
   ```
   git add -A
   git status
   ```
   	
3. Commit local changes.
   ```
   git commit -m "nice descriptive message"
   ```
   
4. Push to **your** branch on the online repo
   ```
   git push origin BRANCH  # Replace "BRANCH" with your GitHub username
   ```


## 4) Submit a Pull Request (to update the `main` branch)

*"Pull Requests" are used to update the `main` branch.  You should submit pull requests frequently, to keep the `main` branch up-to-date, and to facilitate collaboration with others who are using this repo.*

1. Go to the Github website and submit a pull request against your branch.

2. The "owner" of the repo will review your pull request and accept it if everything looks good.  At this moment, the `main` branch and your branch will be identical.
   - After accepting the pull request, the "owner" will also delete the branch (online) from which the pull request was submitted. 
   
3. Whenever the `main` branch is updated, you -- and your repo collaborators -- need to sync your local files (in your branch) with the `main` branch:   
   ```
   cd ~/Projects/IE-482-582/spring2024
   git pull origin main
   ```
   
   This is why you need to perform the steps in Section 2 before you start editing code on your local machine.
   
---

## Troubleshooting

The main issue we are likely to encounter involves two people editing the same file.  For example:
- Suppose Mary and John are collaborating on a project. 
- Mary edits the main `README.md` file, and pushes to the `mary` branch.
- John also edits this `README.md` file in his local `john` branch. 
- Now, Mary submits a pull request, and the owner merges Mary's new `README` into the `main`branch.
- At this point, John's branch is behind the `main` (Mary's branch is even with the `main`).
- If John tries to push his `README` to his branch, and wants to create a pull request, there's going to be a problem...John's `README` is now older than what's in the `main`.

What we want to happen is for John to "pull" the `main` branch into his branch.  Fortunately, Github will recognize that there's a mis-match between the `README` files.  John must now manually edit the `README` to ensure that his changes are incorporated into the `main` version of the file.  Once he completes this task, he may push to his branch and submit a pull request. 


--- 

## Administrative Control of the Main Branch

*To ensure that we don't accidentally/prematurely merge into the `main` branch, we will apply "branch protection rules".  The following steps should only be performed by the "owner" of the repo (probably Murray).*

1. From the repo's main page, select "settings".
2. Go to the "branches" link on the left.
3. From the "branch protection rules" section, choose "add rule".
4. For the "branch name pattern", type `main`.
5. In the "rule settings", check the box for "Restrict who can push to matching branches".  Then, enter the Github userid for the "owner" of the branch (probably "cmurray3").
6. Save your changes by clicking the "create" button.

Now, only the person identified as the owner will be able to update the main branch.  

---    

   
