describe("errors", () => {
    test("invalid tag", () => {
        expect(() => {
            Intl.getCanonicalLocales("EN_US");
        }).toThrowWithMessage(RangeError, "EN_US is not a structurally valid language tag");
    });

    test("root tag", () => {
        expect(() => {
            Intl.getCanonicalLocales("root");
        }).toThrowWithMessage(RangeError, "root is not a structurally valid language tag");
    });

    test("no language tag", () => {
        expect(() => {
            Intl.getCanonicalLocales("Latn");
        }).toThrowWithMessage(RangeError, "Latn is not a structurally valid language tag");
    });

    test("duplicate variant subtags", () => {
        expect(() => {
            Intl.getCanonicalLocales("en-POSIX-POSIX");
        }).toThrowWithMessage(
            RangeError,
            "en-POSIX-POSIX is not a structurally valid language tag"
        );
    });

    test("improperly placed separator", () => {
        expect(() => {
            Intl.getCanonicalLocales("en-");
        }).toThrowWithMessage(RangeError, "en- is not a structurally valid language tag");

        expect(() => {
            Intl.getCanonicalLocales("-en");
        }).toThrowWithMessage(RangeError, "-en is not a structurally valid language tag");

        expect(() => {
            Intl.getCanonicalLocales("en--US");
        }).toThrowWithMessage(RangeError, "en--US is not a structurally valid language tag");
    });

    test("non string or object locale", () => {
        expect(() => {
            Intl.getCanonicalLocales([true]);
        }).toThrowWithMessage(TypeError, "true is neither an object nor a string");
    });
});

describe("normal behavior", () => {
    test("length is 1", () => {
        expect(Intl.getCanonicalLocales).toHaveLength(1);
    });

    test("valid locales", () => {
        expect(Intl.getCanonicalLocales([])).toEqual([]);
        expect(Intl.getCanonicalLocales("EN-US")).toEqual(["en-US"]);
        expect(Intl.getCanonicalLocales(["EN-US"])).toEqual(["en-US"]);
        expect(Intl.getCanonicalLocales(["EN-US", "Fr"])).toEqual(["en-US", "fr"]);
        expect(Intl.getCanonicalLocales("EN-lATN-US")).toEqual(["en-Latn-US"]);
        expect(Intl.getCanonicalLocales("EN-US-POSIX")).toEqual(["en-US-posix"]);
        expect(Intl.getCanonicalLocales("EN-LATN-US-POSIX")).toEqual(["en-Latn-US-posix"]);
    });

    test("duplicate locales", () => {
        expect(Intl.getCanonicalLocales(["EN-US", "en-US", "en-us"])).toEqual(["en-US"]);
    });

    test("non-array object", () => {
        expect(Intl.getCanonicalLocales({})).toEqual([]);
        expect(Intl.getCanonicalLocales({ en: 123 })).toEqual([]);
        expect(Intl.getCanonicalLocales(undefined)).toEqual([]);
        expect(Intl.getCanonicalLocales(true)).toEqual([]);
        expect(Intl.getCanonicalLocales(123)).toEqual([]);
    });
});
