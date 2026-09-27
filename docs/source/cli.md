# suave CLI

The `suave` command wraps the common SUAVE workflows: building and running the Docker
images, building and testing the packages, and running, resuming and listing experiment
campaigns. It does not replace the scripts and launch files described in {doc}`run` and
{doc}`docker`, it calls them.

## Setup

Use any of: `source <repo>/env.sh`, build the workspace and source it (the `suave_cli`
package adds `suave` to `PATH`), or symlink `suave_cli/bin/suave` into a folder on your
`PATH`. The SUAVE images already have it on `PATH`.

## Common commands

    suave docker build                  # build suave-headless:latest (--all adds the GUI images)
    suave docker run                    # start the 'suave' container in the background
    suave docker shell                  # shell inside it, ROS sourced
    suave test suave_runner             # build and test one package
    suave run --config my_runner.yml    # run a campaign
    suave batch start                   # run the campaigns in batch_campaigns.yml
    suave batch resume --latest         # resume the newest batch
    suave campaign resume --latest --config my_runner.yml

Run `suave --help` for everything, and `suave config show` for the active settings.
See `suave_cli/README.md` in the repository for the configuration keys.
