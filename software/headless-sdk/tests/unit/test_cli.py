"""Unit tests for CLI argument parsing (no boot)."""
import pytest

from vayu_headless import cli


def test_serve_defaults():
    args = cli.build_parser().parse_args(["serve"])
    assert args.sub == "serve" and args.alt == -5.0 and args.func is not None


def test_run_course_and_secs():
    args = cli.build_parser().parse_args(
        ["run", "--course", "8,0;8,8", "--secs", "20", "--alt", "-4"])
    assert args.course == "8,0;8,8" and args.secs == 20.0 and args.alt == -4.0


def test_do_collects_command_remainder():
    args = cli.build_parser().parse_args(["do", "fly", "8,0;8,8", "-5", "40"])
    assert args.command == ["fly", "8,0;8,8", "-5", "40"]


def test_missing_subcommand_errors():
    with pytest.raises(SystemExit):
        cli.build_parser().parse_args([])
