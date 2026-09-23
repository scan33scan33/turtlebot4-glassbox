"""
taxonomy.py — the shared 18-class label space, plus how COCO and Objects365
category names map into it.

Both datasets get remapped into these ids (0..17) so the merged YOLO dataset is
consistent. COCO uses exactly these names; Objects365 uses different names
(resolved by name at runtime against the annotation JSON, so we don't rely on
brittle category-index guesses).
"""

TARGET_CLASSES = [
    "chair", "book", "bottle", "dining table", "bowl", "handbag", "backpack",
    "potted plant", "couch", "cell phone", "suitcase", "vase", "sports ball",
    "tv", "dog", "teddy bear", "refrigerator", "bed", "person",
]
ID = {name: i for i, name in enumerate(TARGET_CLASSES)}

# COCO uses identical names -> straight map.
COCO_TO_ID = {name: ID[name] for name in TARGET_CLASSES}

# Objects365 names -> target id. Several are approximate cross-dataset matches:
#   dining table <- "Desk"        (O365 has no dining-table class; Desk is closest)
#   suitcase     <- "Luggage"
#   teddy bear   <- "Stuffed Toy"
#   tv           <- "Monitor/TV"
#   sports ball  <- every ball-type class O365 splits it into
# Names are matched case-insensitively at runtime; any that don't exist in the
# JSON are reported and skipped, so listing extra ball types is harmless.
O365_TO_ID = {
    "Chair": ID["chair"],
    "Book": ID["book"],
    "Bottle": ID["bottle"],
    "Desk": ID["dining table"],
    "Bowl/Basin": ID["bowl"],
    "Handbag/Satchel": ID["handbag"],
    "Backpack": ID["backpack"],
    "Potted Plant": ID["potted plant"],
    "Couch": ID["couch"],
    "Cell Phone": ID["cell phone"],
    "Luggage": ID["suitcase"],
    "Vase": ID["vase"],
    "Monitor/TV": ID["tv"],
    "Dog": ID["dog"],
    "Stuffed Toy": ID["teddy bear"],
    "Refrigerator": ID["refrigerator"],
    "Bed": ID["bed"],
    "Person": ID["person"],
    # all ball types -> "sports ball"
    "Basketball": ID["sports ball"],
    "Soccer": ID["sports ball"],
    "Baseball": ID["sports ball"],
    "Volleyball": ID["sports ball"],
    "American Football": ID["sports ball"],
    "Golf Ball": ID["sports ball"],
    "Tennis": ID["sports ball"],
    "Billiards": ID["sports ball"],
    "Table Tennis ": ID["sports ball"],
}


def norm(s: str) -> str:
    return "".join(s.lower().split())
