# Taken from https://github.com/orf/human_id #

import itertools
import random
from collections.abc import Hashable

__all__ = ["generate_id"]

system_random = random.SystemRandom()


def generate_id(separator="-", seed: Hashable = None, word_count=6) -> str:
    """
    Generate a human readable ID
    :param separator: The string to use to separate words
    :param seed: The seed to use. The same seed will produce the same ID
    :param word_count: The number of words to use. Minimum of 3.
    :return: A human readable ID
    """
    if word_count < 3:
        raise ValueError("word_count cannot be lower than 3")

    random_obj = system_random
    if seed:
        random_obj = random.Random(seed)

    parts = {verbs: 1, adjectives: 1, nouns: 1}

    for _ in range(3, word_count):
        parts[random_obj.choice(list(parts.keys()))] += 1

    parts = itertools.chain.from_iterable(
        random_obj.sample(part, count) for part, count in parts.items()
    )

    return separator.join(parts)


nouns = (
    "time",
    "year",
    "people",
    "way",
    "day",
    "man",
    "thing",
    "woman",
    "life",
    "child",
    "world",
    "school",
    "state",
    "family",
    "student",
    "group",
    "country",
    "problem",
    "hand",
    "part",
    "place",
    "case",
    "week",
    "company",
    "system",
    "program",
    "question",
    "work",
    "government",
    "number",
    "night",
    "point",
    "home",
    "water",
    "room",
    "mother",
    "area",
    "money",
    "story",
    "fact",
    "month",
    "lot",
    "right",
    "study",
    "book",
    "eye",
    "job",
    "word",
    "business",
    "issue",
    "side",
    "kind",
    "head",
    "house",
    "service",
    "friend",
    "father",
    "power",
    "hour",
    "game",
    "line",
    "end",
    "member",
    "law",
    "car",
    "city",
    "community",
    "name",
    "president",
    "team",
    "minute",
    "idea",
    "kid",
    "body",
    "information",
    "back",
    "parent",
    "face",
    "others",
    "level",
    "office",
    "door",
    "health",
    "person",
    "art",
    "war",
    "history",
    "party",
    "result",
    "change",
    "morning",
    "reason",
    "research",
    "girl",
    "guy",
    "moment",
    "air",
    "teacher",
    "force",
    "education",
)

adjectives = (
    "other",
    "new",
    "good",
    "high",
    "old",
    "great",
    "big",
    "american",
    "small",
    "large",
    "national",
    "young",
    "different",
    "black",
    "long",
    "little",
    "important",
    "political",
    "bad",
    "white",
    "real",
    "best",
    "right",
    "social",
    "only",
    "public",
    "sure",
    "low",
    "early",
    "able",
    "human",
    "local",
    "late",
    "hard",
    "major",
    "better",
    "economic",
    "strong",
    "possible",
    "whole",
    "free",
    "military",
    "true",
    "federal",
    "international",
    "full",
    "special",
    "easy",
    "clear",
    "recent",
    "certain",
    "personal",
    "open",
    "red",
    "difficult",
    "available",
    "likely",
    "short",
    "single",
    "medical",
    "current",
    "wrong",
    "private",
    "past",
    "foreign",
    "fine",
    "common",
    "poor",
    "natural",
    "significant",
    "similar",
    "hot",
    "dead",
    "central",
    "happy",
    "serious",
    "ready",
    "simple",
    "left",
    "physical",
    "general",
    "environmental",
    "financial",
    "blue",
    "democratic",
    "dark",
    "various",
    "entire",
    "close",
    "legal",
    "religious",
    "cold",
    "final",
    "main",
    "green",
    "nice",
    "huge",
    "popular",
    "traditional",
    "cultural",
)

verbs = (
    "be",
    "have",
    "do",
    "say",
    "go",
    "can",
    "get",
    "would",
    "make",
    "know",
    "will",
    "think",
    "take",
    "see",
    "come",
    "could",
    "want",
    "look",
    "use",
    "find",
    "give",
    "tell",
    "work",
    "may",
    "should",
    "call",
    "try",
    "ask",
    "need",
    "feel",
    "become",
    "leave",
    "put",
    "mean",
    "keep",
    "let",
    "begin",
    "seem",
    "help",
    "talk",
    "turn",
    "start",
    "might",
    "show",
    "hear",
    "play",
    "run",
    "move",
    "like",
    "live",
    "believe",
    "hold",
    "bring",
    "happen",
    "must",
    "write",
    "provide",
    "sit",
    "stand",
    "lose",
    "pay",
    "meet",
    "include",
    "continue",
    "set",
    "learn",
    "change",
    "lead",
    "understand",
    "watch",
    "follow",
    "stop",
    "create",
    "speak",
    "read",
    "allow",
    "add",
    "spend",
    "grow",
    "open",
    "walk",
    "win",
    "offer",
    "remember",
    "love",
    "consider",
    "appear",
    "buy",
    "wait",
    "serve",
    "die",
    "send",
    "expect",
    "build",
    "stay",
    "fall",
    "cut",
    "reach",
    "kill",
    "remain",
)
