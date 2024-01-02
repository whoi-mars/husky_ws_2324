---
name: Report a bug
about: Create a report to help us improve (but make sure to read all the documentation first)
title: ''
labels: 'needs triage'
assignees: 'umlaeute'

---

### Step 1: Read this

Thanks for your feedback. It is invaluable for making `v4l2loopback` a better software.

To help us making the most of your feedback (so we can e.g. fix bugs more quickly), please make sure to provide the information requested in this template.
Also make sure to remove any non-relevant parts (so we can focus on the essential problem).

Please keep in mind that the development of `v4l2loopback` is done by volunteers.
They are spending their spare time to give you a hopefully nice product and to help you if you have troubles - for free.

Please look through the list of issues (*open* and **closed** ones alike), to see whether you problem has been reported before. Probably you can find a solution to your problem without having to create a new ticket.

#### Seeking Help? Looking for Support?
This issue tracker is meant to track specific bugs in the code (and new features).
However, it is ill-suited as a user support forum.

If you have general questions or problems, please use the `v4l2loopback` tag on Stack Overflow instead:
https://stackoverflow.com/questions/tagged/v4l2loopback


#### Remove Cruft

Please *remove* these instructions (and other non-relevant information) from your report.
If your report looks like a copy of the template, it might get closed immediately.

#### Title
Please chose an appropriate title: "*does not work*" or "*found a bug*" are way too generic.
Try to find a one-liner that says what is not working (e.g. "*module fails to load*").

Also try not to use relative terms.
E.g. "*fails to build with latest kernel*" is bad, because the latest kernel at the time you create the bug report might not be the latest kernel when the problem is being worked on.

#### Accessibility
Sometimes pictures say more.
However, mostly they prevent the use of advanced tools (like "search" or "copy&paste").
And always they prevent people who don't use graphical browser to access the tracker from reading your content.
So, to make the web a better place, we ask you to post *text* rather than *screenshots of text* whenever feasible (pretty much always).


#### Do some research first

Please read the documentation carefully: while it is not very long, it does
contain some information that might help fixing your problem.
If you have trouble compiling the kernel module, double-read the 'DEPENDENCIES'
section in the README.md.

Also make sure to check the issue tracker.
Maybe somebody already reported your problem? Maybe they found a workaround that
could help you.
If you found that somebody already reported your problem, it's *much* better to
add additional information to that report than opening a new one.
If your entire problem has been described and there's nothing to add for you,
consider voting the issue up (using the :+1: emoji; please avoid posts that only
say "me too").



### Step 2: Describe your environment

  * `v4l2loopback` version: _____

          sudo dmesg  | grep -i v4l2loopback

  * kernel version: _____

          uname -a

  * Distribution (+version): _____

          lsb_release -a

### Step 3: Describe the problem:

#### Steps to reproduce:

  1. _____
  2. _____
  3. _____

#### Observed Results:

  * What happened?  This could be a description, log output, etc.

#### Expected Results:

  * What did you expect to happen?

#### Relevant Code:

  ```
  // TODO(you): code here to reproduce the problem
  ```
