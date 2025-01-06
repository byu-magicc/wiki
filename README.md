# MAGICC Public Wiki

Deployed website link: https://byu-magicc.github.io/wiki/

This is the location of the public MAGICC lab wiki, a place to put any useful guides or information for future students and other labmembers. 
There is also a [private wiki](https://github.com/byu-magicc/wiki-private) for more confidential information or anything that doesn't really need to be public.
Please place all appropriate material in that repository instead of this one.

This website is built with Material for MkDocs. The guide below describes how to get the website up and running on a local machine for easy editing. For more extensive documentation, visit the [official documentation](https://squidfunk.github.io/mkdocs-material/).

## Install mkdocs

``` bash
pip install mkdocs-material
```

## Run the mkdocs Server

Type `mkdocs serve` in the root directory. It should report to you something like:

``` bash
[I 170728 07:49:47 server:271] Serving on http://127.0.0.1:8000
[I 170728 07:49:47 handlers:58] Start watching changes
```

This means that mkdocs is hosting a webpage for you on http://127.0.0.1:8000. Navigate to that page in your web browser.

Now, as you make changes to the documentation, you should be able to see it on your browser. Just hit reload from time to time to see your changes.

## Adding Pages
To add a new page to the documentation, just take a look at the mkdocs.yml file in the root of the firmware directory. You should be able to figure it out from there.

## Adding LaTeX
The syntax for adding LaTeX math inline is `\( x \)`, which renders as x. For adding a block, it's

``` latex
$$ E = mc^2 $$
```

## Publishing changes on website

Any changes that are pushed to the main branch will automatically be applied to the website. Just submit a pull request!
