import math
import random

from tag2nav_demo.core import (Anchor, Pose, greedy_layout, layout_utility,
                               relocalization_trial, visible_probability)


def test_visibility_rejects_behind_camera():
    a = Anchor(1, "a", 2.0, 0.0, math.pi)
    assert visible_probability(Pose(0, 0, 0), a, 4.0, math.pi / 3) > 0
    assert visible_probability(Pose(0, 0, math.pi), a, 4.0, math.pi / 3) == 0


def test_greedy_selects_useful_anchor():
    anchors = {1: Anchor(1, "near", 2, 0, 0), 2: Anchor(2, "far", 20, 0, 0)}
    poses = [(Pose(0, 0, 0), 1.0)]
    assert greedy_layout(anchors, poses, 1, 5.0, math.pi / 2) == {1}


def test_more_visible_tags_do_not_reduce_utility():
    anchors = {1: Anchor(1, "a", 2, 0, 0), 2: Anchor(2, "b", 3, 0, 0)}
    poses = [(Pose(0, 0, 0), 1.0)]
    assert layout_utility({1, 2}, anchors, poses, 5, math.pi / 2) >= layout_utility({1}, anchors, poses, 5, math.pi / 2)
