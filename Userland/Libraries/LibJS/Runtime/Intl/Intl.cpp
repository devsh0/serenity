/*
 * Copyright (c) 2021, Linus Groh <linusg@serenityos.org>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <LibJS/Runtime/Array.h>
#include <LibJS/Runtime/GlobalObject.h>
#include <LibJS/Runtime/Intl/AbstractOperations.h>
#include <LibJS/Runtime/Intl/DisplayNamesConstructor.h>
#include <LibJS/Runtime/Intl/Intl.h>

namespace JS::Intl {

// 8 The Intl Object, https://tc39.es/ecma402/#intl-object
Intl::Intl(GlobalObject& global_object)
    : Object(*global_object.object_prototype())
{
}

void Intl::initialize(GlobalObject& global_object)
{
    Object::initialize(global_object);

    auto& vm = this->vm();

    // 8.1.1 Intl[ @@toStringTag ], https://tc39.es/ecma402/#sec-Intl-toStringTag
    define_direct_property(*vm.well_known_symbol_to_string_tag(), js_string(vm, "Intl"), Attribute::Configurable);

    u8 attr = Attribute::Writable | Attribute::Configurable;
    define_direct_property(vm.names.DisplayNames, global_object.intl_display_names_constructor(), attr);

    define_native_function(vm.names.getCanonicalLocales, get_canonical_locales, 1, attr);
}

// 8.3.1 Intl.getCanonicalLocales ( locales ), https://tc39.es/ecma402/#sec-intl.getcanonicallocales
JS_DEFINE_NATIVE_FUNCTION(Intl::get_canonical_locales)
{
    auto locales = vm.argument(0);

    // 1. Let ll be ? CanonicalizeLocaleList(locales).
    auto locale_list = canonicalize_locale_list(global_object, locales);
    if (vm.exception())
        return {};

    MarkedValueList marked_locale_list { vm.heap() };
    marked_locale_list.ensure_capacity(locale_list.size());
    for (auto& locale : locale_list)
        marked_locale_list.append(js_string(vm, move(locale)));

    // 2. Return CreateArrayFromList(ll).
    return Array::create_from(global_object, marked_locale_list);
}

}
